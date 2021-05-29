#include "Estimator.hpp"
#include <opencv2/core/eigen.hpp>

namespace vio{
Estimator::Estimator(std::string configFile) {
  cfg_ = new Config(configFile);
  cam_ = new Camera(cfg_);
  init_ = new Initializator(cfg_,cam_);
  feaTrcker_ = new FeatureTracker(cfg_);
  pnpSolver_ = new PnpSolver(cfg_);
  state = EstState::Waiting;
  preInteNow_ = nullptr;
  cv::cv2eigen(cfg_->extrinsicParam_.Rbc,Rbc_);
  cv::cv2eigen(cfg_->extrinsicParam_.tbc,tbc_);
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

void Estimator::update(FramePtr frame,bool trackEnable) {
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
  checkParallex(frame);
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
      int i = 0;
      for (auto ref : slideWindows_) {
        i++;
        if (state != EstState::Runing) {
          if (init_->initPoseAndMap(ref,frame,fsm_)) {
            state = EstState::Runing;
          }
        } else if (i < slideWindows_.size()) {
          if (!estimatePose(ref)) {
            reset();
            state = EstState::Waiting;
            break;
          }
          std::cout << "twc = " << ref->WtC().t() << std::endl;
        }
      }
      
      break;
    }
  case EstState::Runing:
    if (fsm_.getFeatureSize() > 5 && estimatePose(slideWindows_.back()) && checkPose()) {
      updateFeature();
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
}

void Estimator::slideWindow() {
  if (slideWindows_.size() > 1 && !removeOldKeyFrame_) { //remove second new frame no wait
    std::vector<FramePtr>::iterator it = slideWindows_.end() - 1;
    fsm_.removeFrame(*it);
    slideWindows_.erase(it);
  }
  if (slideWindows_.size() > WINSIZE && removeOldKeyFrame_) { //remove old frame until slidewindow is full
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

void Estimator::checkParallex(FramePtr frame) {
  if (slideWindows_.empty()) {
    return;
  }
  std::vector<uint64> idVec;
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

bool Estimator::estimatePose(FramePtr frame) {
  std::vector<cv::Point2f> matchedNormalizedUV;
  std::vector<cv::Point3f> matchedPts3D;
  std::vector<uint64_t> matchedIds;
  fsm_.featureMatching(cam_,frame,matchedIds,matchedNormalizedUV,matchedPts3D);
  if (matchedPts3D.size() < 5) {
    printf("[EstimatePose]: matched points is too few!\n");
    return false;
  } 
  cv::Mat rcw,CtW,Rcw;
  std::vector<int> inliers;
  if (pnpSolver_->solveByPnp(matchedNormalizedUV,matchedPts3D,cam_->fx(),rcw,CtW,inliers)) {
    cv::Mat Rwc,WtC;
    cv::Rodrigues(rcw,Rcw);
    Rwc = Rcw.t();
    WtC = - Rwc * CtW;
    frame->setPoseInWorld(Rwc,WtC);
    return true;
  } 
  std::cout << "[EstimatePose]:Failure!" << std::endl;
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

void Estimator::updateFeature() {
  //step1: add new features
  FramePtr curFramePtr = slideWindows_.back();
  std::map<uint64,cv::Point2f> &corners = curFramePtr->getCorners();
  std::map<uint64,Feature>& features = fsm_.getFeatureMap(); 
  std::vector<uint64> idx;
  std::vector<cv::Point2f> ptVec,proPtVec;
  std::vector<cv::Point3f> p3DVec;
  for (auto it = corners.begin();it != corners.end(); it++) {
    const uint64 id = it->first;
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
    if (cv::norm(proPtVec[i] - ptVec[i]) > cfg_->estimatorParam_.ReprojectPixelErr) {
      corners.erase(idx[i]);
      fsm_.updateBadCount(idx[i]);
    } else {
      fsm_.addFeature(idx[i],curFramePtr);
    }
  }

  //step2: remove untracked and bad features
  for (auto it = features.begin(); it != features.end();) {
    if (!it->second.isInFrame(curFramePtr)) {
      fsm_.removeFeature(it++);
      continue;
    }
    if (it->second.getBadCount() >= 3) {
      fsm_.removeFeature(it++);
      cv::Point3f pt3d = it->second.getPts3DInWorld();
      printf("[Features]:Remove %lud feature[%f,%f] for bad count > 3!\n",it->first,pt3d.x/pt3d.z,pt3d.y/pt3d.z);
      continue;
    }
    it++;
  }
}

void Estimator::calCameraRotationMatrix(double lastT, double curT, cv::Mat &R_cur_last) {
  if (lastT > curT || curT - lastT > 1.0) {
    printf("[calCameraRotationMatrix]:last timestamp: %12.4f and current timestamp: %12.4f have big interval or bad sequence!\n",lastT,curT);
    return;
  }
  std::lock_guard<std::mutex> imuLck(m_imu_);
  std::map<double,IMU>::const_iterator itBegin = imuMeas_.getLowIter(lastT);
  std::map<double,IMU>::const_iterator itEnd = imuMeas_.getHighIter(curT);

  if (itBegin == imuMeas_.measMap_.end() || itEnd == imuMeas_.measMap_.end()) {
    printf("[calCameraRotationMatrix]:imu infos not includes data from %12.4f to %12.4f!\n",lastT,curT);
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

