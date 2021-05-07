#include "Estimator.hpp"

namespace ov {
Estimator::Estimator(std::string configFile) {
  cfg_ = new Config(configFile);
  cam_ = new Camera(cfg_);
  init_ = new Initializator(cfg_,cam_);
  feaTrcker = new FeatureTracker(cfg_);
  state = EstState::Waiting;
  reset();
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
    if (slideWindows_.size() >= 2) {
      state = EstState::Initing;
    }
    break;
  case EstState::Initing:
    for (auto ref : slideWindows_) {
      if (init_->initPoseAndMap(ref.get(),frame.get(),fsm_)) {
        state = EstState::Runing;
        break;
      }
    }
    break;
  case EstState::Runing:
    if (fsm_.getFeatureSize() > 5 && estimatePose() && checkPose()) {
      updateFeature();
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
  std::vector<cv::Point2f> matchedNormalizedUV;
  std::vector<cv::Point3f> matchedPts3D;
  fsm_.featureMatching(cam_,curFramePtr,matchedNormalizedUV,matchedPts3D);
  if (matchedPts3D.size() < 5) {
    return false;
  } 
  cv::Mat rcw,CtW,Rcw;
  std::vector<int> inlier;
  if (pnpSolver_.solveByPnp(matchedNormalizedUV,matchedPts3D,cam_->fx(),rcw,CtW,inlier,0.8)) {
    cv::Mat Rwc,WtC;
    cv::Rodrigues(rcw,Rcw);
    Rwc = Rcw.t();
    WtC = - Rwc * CtW;
    curFramePtr->setPoseInWorld(Rwc,WtC);
    return true;
  } 
  std::cout << "[Estimator-Pose]:Failure!" << std::endl;
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
  for (auto it = features.begin(); it != features.end();) {
    if (!it->second.isInFrame(curFramePtr.get())) {
      fsm_.removeFeature(it++);
      continue;
    }
    it++;
  }
}


}

