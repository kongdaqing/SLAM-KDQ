#include "Estimator.hpp"
#include <opencv2/core/eigen.hpp>
#include "BASolver.hpp"
#include "fileSystem.hpp"
#include "tictoc.hpp"
#include "BSplineX.hpp"
namespace vio{
Estimator::Estimator(std::string configFile) {
  cfg_ = new Config(configFile);
  cam_ = new Camera(cfg_);
  init_ = new Initializator(cfg_,cam_);
  feaTrcker_ = new FeatureTracker(cfg_);
  pnpSolver_ = new PnpSolver(cfg_);
  baSolver_ = new BAG2O();
  state = EstState::Waiting;
  preInteNow_ = nullptr;
  moduleName_ = "Estimator";
  cv::cv2eigen(cfg_->extrinsicParam_.Rbc,Rbc_);
  cv::cv2eigen(cfg_->extrinsicParam_.tbc,tbc_);
  FileSystem::fInfo = fopen(cfg_->estimatorParam_.LogName.c_str(),"wb+");
  FileSystem::printInfos(LogType::Info,moduleName_ + "|Pose","timestamp,allCost,trackCost,initCost,pnpCost,baCost,px,py,pz,qw,qx,qy,qz");
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
      tim.tic();
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
          for(auto w : slideWindows_) {
            updateSlideAccelAndPose(w);
          }
        } else {
          reset();
          state = Waiting;
        }
      } else {
        reset();
        state = Waiting;
      }
      costTime[1] = tim.toc();
      break;
    }
    case EstState::Runing:
      tim.tic();
      if (slideWindows_.size() > 1 && fsm_.getFeatureSize() > 5 && estimatePose(slideWindows_.back()) && checkPose()) {
        //updateFeature must be first than ba,otherwize curframe not be update
        updateFeature(slideWindows_.back());
        updateSlideAccelAndPose(slideWindows_.back());
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
    //目前认为移除最老帧比较合理
    if (removeOldKeyFrame_ || true) {
      fsm_.removeFrame(slideWindows_.front());
      slideWindows_.erase(slideWindows_.begin());
    } else {
      std::vector<FramePtr>::const_iterator secondNewFrame = slideWindows_.end() - 2;
      fsm_.removeFrame(*secondNewFrame);
      slideWindows_.erase(secondNewFrame);
    }
  }
}

void Estimator::reset() {
  goodExcitationCount = 0;
  fsm_.reset();
  slideWindows_.clear();
  slideAccel_.clear();
  slidePose_.clear();
  imuMeas_.clear();
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
  if (slideWindows_.size() < WINSIZE || !cfg_->estimatorParam_.BundleAdjustment) {
    return;
  }
  if (baSolver_->constructWindowFrameOptimize(slideWindows_,fsm_,2.0/cam_->fx())) {
    baSolver_->updatePoseAndMap(slideWindows_, fsm_);
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

void Estimator::updateSlideAccelAndPose(FramePtr curFrame) {
  if (imuMeas_.empty() || curFrame == nullptr || slidePose_.count(curFrame->timestamp_)) {
    return;
  }
  cv::Mat t = curFrame->WtC();
  Eigen::Vector3d twc;
  cv::cv2eigen(t,twc);
  slidePose_[curFrame->timestamp_] = twc;
  Eigen::Vector3d sumAccel;
  std::vector<Eigen::Vector3d> accelVec;
  sumAccel.setZero();
  //update accel map that timestamp is smaller than current timestamp
  for (auto it = imuMeas_.measMap_.rbegin(); it != imuMeas_.measMap_.rend(); it++) {
    double imuTimes = it->second.timestamp_;
    if (imuTimes > curFrame->timestamp_) {
      continue;
    }
    if (slideAccel_.count(imuTimes) || imuTimes < (curFrame->timestamp_ - 1.0)) {
      break;
    }
    slideAccel_[imuTimes] = it->second.acc_;
    sumAccel += it->second.acc_;
    accelVec.push_back(it->second.acc_);
  }
  //Check whether accel excitation is enough
  if (accelVec.size() > 1) {
    Eigen::Vector3d averAcc = sumAccel / accelVec.size();
    double sumErr = 0;
    for (auto a : accelVec) {
      sumErr += (a - averAcc).norm();
    }
    double standErr = sumErr / (accelVec.size() - 1);
    if (standErr > 0.1) {
      goodExcitationCount++;
    }
  }

  //slide old pose and accel
  if (slidePose_.size() > S) {
    while (!slideAccel_.empty() && slideAccel_.begin()->first < slidePose_.begin()->first) {
      slideAccel_.erase(slideAccel_.begin());
    }
    slidePose_.erase(slidePose_.begin());
    goodExcitationCount--;
    calWindowsAccelByBSpline();
  }
}

void Estimator::calWindowsAccelByBSpline() {
  if (slidePose_.size() != S || goodExcitationCount < S/2 && false) {
    return;
  }
  Eigen::Matrix<double,S,1> x;
  Eigen::Matrix<double,S,D> y;
  std::ofstream samplesFile("samples.csv",std::ios::app | std::ios::out);
  samplesFile << "==================================================" << std::endl;
  int cnt = 0;
  for (std::map<double,Eigen::Vector3d>::const_iterator it = slidePose_.begin();it != slidePose_.end(); it++) {
    x(cnt,0) = it->first;
    y.row(cnt) = it->second;
    cnt++;
    samplesFile << it->first << "," << it->second.x() << "," << it->second.y() << "," << it->second.z() << std::endl;
  }
  samplesFile << "----------------------------------------------------" << std::endl;
  BSplineX<S,D,K> splinex(x,y);
  std::vector<Eigen::Vector3d> splineAcc,imuAccel;
  double t0 = x(1,0);
  double t1 = x(S-1,0);
  for (auto it = slideAccel_.begin(); it != slideAccel_.end(); it++) {
    if (it->first < t0) {
      continue;
    }
    if (it->first > t1) {
      break;
    }
    Eigen::Matrix<double, 1, D> pos,vel,acc;
    if (splinex.getSecondDifference(it->first, acc)) {
      splinex.getEvalValue(it->first,pos);
      splinex.getFirstDifference(it->first,vel);
      splineAcc.push_back(acc.transpose());
      imuAccel.push_back(it->second);
      samplesFile << it->first << "," << pos.x() << "," << pos.y() << "," << pos.z() << ","
                  << vel.x() << "," << vel.y() << "," << vel.z() << ","
                  << acc.x() << "," << acc.y() << "," << acc.z() << "," << acc.norm() << ","
                  << it->second.x() << "," << it->second.y() << "," << it->second.z() << "," << it->second.norm() << std::endl;
    }
  }
  calScaleAndR0(splineAcc,imuAccel);
}

void Estimator::calScaleAndR0(const std::vector<Eigen::Vector3d> &splineAcc,
                              const std::vector<Eigen::Vector3d> &imuAcc) {


}

}

