#include "Estimator.hpp"
#include <opencv2/core/eigen.hpp>
#include "BASolver.hpp"
#include "fileSystem.hpp"
#include "tictoc.hpp"
namespace vio{
Estimator::Estimator(std::string configFile) {
  cfg_ = new Config(configFile);
  cam_ = new Camera(cfg_);
  init_ = new Initializator(cfg_,cam_);
  feaTrcker_ = new FeatureTracker(cfg_);
  pnpSolver_ = new PnpSolver(cfg_);
  state = EstState::Waiting;
  preInteNow_ = nullptr;
  moduleName_ = "Estimator";
  cv::cv2eigen(cfg_->extrinsicParam_.Rbc,Rbc_);
  cv::cv2eigen(cfg_->extrinsicParam_.tbc,tbc_);
  FileSystem::fInfo = fopen(cfg_->estimatorParam_.LogName.c_str(),"wb+");
  reset();
}

Estimator::~Estimator() {
  reset();
  delete cfg_;
  delete cam_;
  delete init_;
  delete feaTrcker_;
  delete pnpSolver_;
}

void Estimator::getCurrentPose(Eigen::Vector3d &t, Eigen::Quaterniond &q) const {
  cv::Mat Rwc,twc;
  if (getCurrentPose(Rwc,twc)) {
    cv::cv2eigen(twc,t);
    Eigen::Matrix3d R;
    cv::cv2eigen(Rwc,R);
    q = Eigen::Quaterniond(R).normalized();
  }
}

void Estimator::update(FramePtr frame,bool trackEnable) {
  Tictoc tim("estimator"),allTim("all");
  tim.tic();
  allTim.tic();
  double costTime[4] = {0.};
  std::lock_guard<std::mutex> lock(m_filter_);
  FramePtr lastFramePtr = nullptr;
  if (!slideWindows_.empty()) {
    lastFramePtr = slideWindows_.back();
  }
  if (trackEnable) {
    cv::Mat R_cur_last;
    if (lastFramePtr != nullptr)
      calCameraRotationMatrix(lastFramePtr->timestamp_,frame->timestamp_,R_cur_last);
    feaTrcker_->detectAndTrackFeature(lastFramePtr, frame, R_cur_last);
  }
  costTime[0] = tim.toc();
  tim.tic();
  isKeyframe(frame);
  slideWindows_.push_back(frame);
  poseUpdateFlg_ = false;
  switch (state)
  {
    case EstState::Waiting:
      if (slideWindows_.size() >= 2) {
        state = EstState::Initing;
      }
      break;
    case EstState::Initing:
    {
      //KDQ:初始化的时候第一帧选择很重要，如果第一帧和当前帧匹配带你过少，则应该删除第一帧
      FramePtr refFrame = slideWindows_.front();
      if (refFrame->getMatchedFeatureSize(frame.get()) < cfg_->iniParam_.minMatchedFeatNum) {
        slideWindows_.erase(slideWindows_.begin());
      }

      if (slideWindows_.size() < 4) {
        break;
      }
      size_t endId = slideWindows_.size() - 1;
      int initCnt = 1;
      if (init_->initPoseAndMap(slideWindows_[0],slideWindows_[endId],fsm_)) {
        for (;initCnt < endId;initCnt++) {
          if (estimatePose(slideWindows_[initCnt])) {
            updateFeature(slideWindows_[initCnt]);
          } else {
            break;
          }
        }
      } else {
        break;
      }
      if (initCnt == endId) {
        //三角化所有的特征点
        fsm_.triangulateAll();
        BAG2O basolver;
        if (basolver.constructWindowFrameOptimize(slideWindows_,fsm_,2.0/cam_->fx())) {
          basolver.updatePoseAndMap(slideWindows_,fsm_);
          state = Runing;
          costTime[1] = tim.toc();
        } else {
          reset();
          state = Waiting;
        }
      } else {
        reset();
        state = Waiting;
      }
      break;
    }
    case EstState::Runing:
      tim.tic();
      if (slideWindows_.size() > 1 && fsm_.getFeatureSize() > 5 && estimatePose(slideWindows_.back()) && checkPose()) {
        //updateFeature must be first than ba,otherwize curframe not be update
        updateFeature(slideWindows_.back());
        costTime[2] = tim.toc();
        tim.tic();
        bundleAdjustment();
        costTime[3] = tim.toc();
        poseUpdateFlg_ = true;
      } else {
        reset();
        state = EstState::Waiting;
      }
      break;
    default:
      break;
  }
  slideWindow();
  if (slideWindows_.empty()) {
    return;
  }
  double allCost = allTim.toc();
  Eigen::Vector3d twc = Eigen::Vector3d::Zero();
  Eigen::Quaterniond qwc;
  qwc.setIdentity();
  getCurrentPose(twc,qwc);
  FileSystem::printInfos(LogType::Info,moduleName_ + "|Pose","%12.4f,%d,"
                                                             "%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,"
                                                             "%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,%3.4f",
                         slideWindows_.back()->timestamp_,state,
                         allCost,costTime[0],costTime[1],costTime[2],costTime[3],
                         twc.x(),twc.y(),twc.z(),qwc.w(),qwc.x(),qwc.y(),qwc.z());
}

void Estimator::slideWindow() {
  if (slideWindows_.size() > WINSIZE ) { //remove old frame until slidewindow is full
    fsm_.removeFrame(slideWindows_.front());
    slideWindows_.erase(slideWindows_.begin());
  }
}

void Estimator::reset() {
  fsm_.reset();
  slideWindows_.clear();
  feaTrcker_->reset();
  poseUpdateFlg_ = false;
  removeOldKeyFrame_ = true;
}

void Estimator::isKeyframe(FramePtr frame) {
  if (slideWindows_.empty()) {
    return;
  }
  std::vector<uint64_t> idVec;
  std::vector<cv::Point2f> refFeatures,curFeatures;
  float averParallex = 0;
  frame->getMatchedFeatures(slideWindows_.back().get(),idVec,refFeatures,curFeatures,averParallex);
  if (curFeatures.size() < cfg_->estimatorParam_.KeyFrameMatchedPoints ||
      averParallex > cfg_->estimatorParam_.KeyFrameParallexThreshold) {
    removeOldKeyFrame_ = true;
  } else {
    removeOldKeyFrame_ = false;
  }
}

void Estimator::bundleAdjustment() {
  if (slideWindows_.size() < WINSIZE) {
    return;
  }
  BAG2O baSolver;
  if (baSolver.constructWindowFrameOptimize(slideWindows_,fsm_,2.0/cam_->fx())) {
    baSolver.updatePoseAndMap(slideWindows_, fsm_);
  }
}

bool Estimator::estimatePose(FramePtr frame) {
  std::vector<cv::Point2f> matchedNormalizedUV;
  std::vector<cv::Point3f> matchedPts3D;
  std::vector<uint64_t> matchedIds;
  fsm_.featureMatching(frame,matchedIds,matchedNormalizedUV,matchedPts3D);
  if (matchedPts3D.size() < 5) {
    FileSystem::printInfos(LogType::Error,moduleName_ + "|EstimatePose","Matched points is too few less than 5!");
    return false;
  }
  cv::Mat rcw,CtW,Rcw;
  slideWindows_[slideWindows_.size()-2]->getInversePose(rcw,CtW);
  std::vector<int> inliers;
  if (pnpSolver_->solveByPnp(matchedNormalizedUV,matchedPts3D,cam_->fx(),rcw,CtW,inliers)) {
    cv::Mat Rwc,WtC;
    cv::Rodrigues(rcw,Rcw);
    Rwc = Rcw.t();
    WtC = - Rwc * CtW;
    frame->setPoseInWorld(Rwc,WtC);
    return true;
  }
  FileSystem::printInfos(LogType::Error,moduleName_ + "|EstimatePose","PnPSolver failure!");
  return false;
}


bool Estimator::checkPose() {
  if (slideWindows_.size() < 2) {
    return true;
  }
  FramePtr curFramePtr = slideWindows_.back();
  FramePtr oldestFramePtr = slideWindows_.front();
  double dt = curFramePtr->timestamp_ - oldestFramePtr->timestamp_;
  cv::Mat cur_wtc = curFramePtr->WtC();
  cv::Mat old_wtc = oldestFramePtr->WtC();
  if (cur_wtc.size() == old_wtc.size() && cur_wtc.type() == old_wtc.type()) {
    cv::Mat dT = cur_wtc - old_wtc;
    cv::Mat dtNorm = dT.t() * dT;
    double spd = dtNorm.at<double>(0,0) / dt;
    if (!isfinite(spd) && spd > 20) {
      return false;
    }
  }
  return true;
}

void Estimator::updateFeature(FramePtr curFramePtr) {
  //step1: add new features
  std::map<uint64_t,cv::Point2f> &corners = curFramePtr->getCorners();
  std::map<uint64_t,Feature>& features = fsm_.getFeatureMap();
  std::vector<uint64_t> idx;
  std::vector<cv::Point2f> ptVec,proPtVec;
  std::vector<cv::Point3f> p3DVec;
  for (auto it = corners.begin();it != corners.end(); it++) {
    const uint64_t id = it->first;
    cv::Point2f pt = it->second;
    if (features.count(id) && features[id].valid3D()) {
      ptVec.push_back(pt);
      p3DVec.push_back(features[id].getPts3DInWorld());
      idx.push_back(id);
    } else {
      fsm_.addFeature(id,curFramePtr);
    }
  }
  cv::Mat Rcw,CtW,rcw;
  curFramePtr->getInversePose(Rcw,CtW);
  cv::Rodrigues(Rcw,rcw);
  cam_->project(p3DVec,proPtVec,rcw,CtW);
  for(size_t i = 0; i < idx.size(); i++) {
    float repErr = cv::norm(proPtVec[i] - ptVec[i]);
    if (repErr > cfg_->estimatorParam_.ReprojectPixelErr) {
      corners.erase(idx[i]);
      fsm_.updateBadCount(idx[i]);
    } else {
      fsm_.addFeature(idx[i],curFramePtr);
      if (repErr < 0.5) {
        fsm_.updateGoodCount(idx[i]);
      }
    }
  }

  //step2: remove untracked and bad features
  for (auto it = features.begin(); it != features.end();) {
    if (it->second.getTrackCount() == 0) {
      fsm_.removeFeature(it++);
      continue;
    }
    if (it->second.getBadCount() >= 3) {
      cv::Point3f pt3d = it->second.getPts3DInWorld();
      FileSystem::printInfos(LogType::Warning,moduleName_ + "|UpdateMap","Remove %lu feature[%f,%f,%f] for bad count > 3!\n",it->first,pt3d.x,pt3d.y,pt3d.z);
      fsm_.removeFeature(it++);
      continue;
    }
    it++;
  }
}

void Estimator::calCameraRotationMatrix(double lastT, double curT, cv::Mat &R_cur_last) {
  if (lastT > curT || curT - lastT > 1.0) {
    FileSystem::printInfos(LogType::Warning,moduleName_ + "|calCameraRotationMatrix","[calCameraRotationMatrix]:last timestamp: %12.4f and current timestamp: %12.4f have big interval or bad sequence!",lastT,curT);
    return;
  }
  std::lock_guard<std::mutex> imuLck(m_imu_);
  std::map<double,IMU>::const_iterator itBegin = imuMeas_.getLowIter(lastT);
  std::map<double,IMU>::const_iterator itEnd = imuMeas_.getHighIter(curT);

  if (itBegin == imuMeas_.measMap_.end() || itEnd == imuMeas_.measMap_.end()) {
    FileSystem::printInfos(LogType::Warning,moduleName_ + "|calCameraRotationMatrix","Imu infos not includes data from %12.4f to %12.4f!",lastT,curT);
    return;
  }
  IMU lastImu;
  if (preInteNow_ != nullptr)
    delete preInteNow_;
  preInteNow_ = new PreIntegration(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
  std::map<double,IMU>::const_iterator it = itBegin,lastIt = itBegin;
  while (it != itEnd) {
    IMU imu(it->second.timestamp_,it->second.acc_,it->second.gyro_);
    if (it != itBegin) {
      preInteNow_->midIntegration(imu);
    }
    lastImu = imu;
    lastIt = it;
    it++;
    if (lastIt == itBegin ) {
      IMU nexImu(it->second.timestamp_,it->second.acc_,it->second.gyro_);
      Eigen::Vector3d acc = (lastImu.acc_ * (lastT - lastImu.timestamp_) + nexImu.acc_ * (nexImu.timestamp_ - lastImu.timestamp_)) / (nexImu.timestamp_ - lastT);
      Eigen::Vector3d gyr = (lastImu.gyro_ * (lastT - lastImu.timestamp_) + nexImu.gyro_ * (nexImu.timestamp_ - lastImu.timestamp_)) / (nexImu.timestamp_ - lastT);
      IMU nowImu(lastT,acc,gyr);
      preInteNow_->midIntegration(nowImu);
    } else if (it == itEnd) {
      IMU nexImu(it->second.timestamp_,it->second.acc_,it->second.gyro_);
      Eigen::Vector3d acc = (lastImu.acc_ * (curT - lastImu.timestamp_) + nexImu.acc_ * (nexImu.timestamp_ - curT)) / (nexImu.timestamp_ - lastImu.timestamp_);
      Eigen::Vector3d gyr = (lastImu.gyro_ * (curT - lastImu.timestamp_) + nexImu.gyro_ * (nexImu.timestamp_ - curT)) / (nexImu.timestamp_ - lastImu.timestamp_);
      IMU nowImu(curT,acc,gyr);
      preInteNow_->midIntegration(nowImu);
    }
  }
  preInteNow_->fix();
  Eigen::Matrix3d R_lastB_curB = preInteNow_->rotationMatrix();
  Eigen::Matrix3d R = (R_lastB_curB * Rbc_).transpose() * Rbc_;
  cv::Mat cvR = (cv::Mat_<float>(3,3) << R(0,0), R(0,1), R(0,2),
    R(1,0), R(1,1), R(1,2),
    R(2,0), R(2,1), R(2,2));
  cvR.copyTo(R_cur_last);
}

}

