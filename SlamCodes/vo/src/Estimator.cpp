#include "Estimator.hpp"

namespace ov {
Estimator::Estimator(std::string configFile) {
  cfg_ = new Config(configFile);
  cam_ = new Camera(cfg_);
  init_ = new Initializator(cfg_,cam_);
  feaTrcker = new FeatureTracker(cfg_);
}

Estimator::~Estimator() {
  reset();
  delete cfg_;
  delete cam_;
  delete init_;
  delete feaTrcker;
}

void Estimator::update(FramePtr frame) {
  bool trackEnable = false;
  FramePtr lastFramePtr = nullptr;
  if (!slideWindows_.empty()) {
    lastFramePtr = slideWindows_.back();
  }
  if (trackEnable) {
    feaTrcker->detectAndTrackFeature(lastFramePtr,frame);
  } 
  slideWindows_.push_back(frame);
  switch (state)
  {
  case EstState::Waiting:
    reset();
    if (slideWindows_.size() >= 2) {
      state = EstState::Initing;
    }
    break;
  case EstState::Initing:
    if (init_->initPoseAndMap(lastFramePtr.get(),frame.get(),fsm_)) {
      state = EstState::Runing;
    } else {
      state = EstState::Waiting;
    }
    break;
  case EstState::Runing:
    if (estimatePose() && checkPose()) {
      updateFeature();
    } else {
      state = EstState::Waiting;
    }
    break;
  default:
    break;
  }
  slideWindow();
}

void Estimator::slideWindow() {
  if (slideWindows_.size() > WINSIZE) {
    fsm_.removeFrame(slideWindows_.front().get());
    slideWindows_.erase(slideWindows_.begin());
  }
}

void Estimator::reset() {
  fsm_.reset();
  slideWindows_.clear();
}


bool Estimator::estimatePose() {
  Frame* curFramePtr = slideWindows_.back().get();
  std::vector<cv::Point2f> matchedUV;
  std::vector<cv::Point3f> matchedPts3D;
  fsm_.featureMatching(curFramePtr,matchedUV,matchedPts3D);
  if (matchedPts3D.size() < 5) {
    return false;
  } 
  cv::Mat R,t;
  std::vector<uchar> inlier;
  if (pnpSolver_.solveByPnp(matchedUV,matchedPts3D,cam_->fx(),R,t,inlier)) {
    curFramePtr->setPoseInWorld(R,t);
    return true;
  } 
  return false;
}

bool Estimator::checkPose() {
  FramePtr curFramePtr = *slideWindows_.end();
  FramePtr oldestFramePtr = *slideWindows_.begin();
  double dt = curFramePtr->timestamp_ - oldestFramePtr->timestamp_;
  double spd = cv::norm(curFramePtr->WtC() - oldestFramePtr->WtC()) / dt;
  if (!isfinite(spd) && spd > 20) {
    return false;
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
      fsm_.addFeature(id,curFramePtr.get());
    }
  }
  cv::Mat Rcw,CtW,rcw;
  curFramePtr->getInversePose(Rcw,CtW);
  cv::Rodrigues(Rcw,rcw);
  cam_->project(p3DVec,proPtVec,rcw,CtW);
  for(size_t i = 0; i < idx.size(); i++) {
    if (cv::norm(proPtVec[i] - ptVec[i]) > 3.0) {
      corners.erase(idx[i]);
    } else {
      fsm_.addFeature(idx[i],curFramePtr.get());
    }
  }
  //step2: remove untracked features
  for (auto it = features.begin(); it != features.end(); it++) {
    if (!it->second.isInFrame(curFramePtr.get())) {
      fsm_.removeFeature(it);
    }
  }
}


}

