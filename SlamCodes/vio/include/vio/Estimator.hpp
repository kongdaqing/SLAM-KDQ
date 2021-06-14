#pragma once
#include <fstream>
#include "FeatureManager.hpp"
#include "Initializator.hpp"
#include "Config.hpp"
#include "PnpSolver.hpp"
#include "FeatureTracker.hpp"
#include "MeasurementTimeline.hpp"
#include "PreIntegration.hpp"

namespace vio{
enum EstState {
  Waiting,
  Initing,
  Runing
};

class Estimator {
 public:
  std::string cameraName = "cam";
  std::string pointsName = "corners";
  /** \brief construct function
   */ 
  Estimator(std::string configFile);
  
  /** \brief unconstruct function
   */ 
  ~Estimator();

  /** \brief reset system
   */ 
  void reset();

  /** \brief estimator update 
   * @param FramePtr    --- frame shared_ptr 
   * @param trackEnable --- enable track features
   */
  void update(FramePtr frame,bool trackEnable);

  /** \brief get estimator state
   */ 
  inline EstState getEstimatorState() const{
    return state;
  }

  /** \brief get current pose
   */ 
  bool getCurrentPose(cv::Mat& Rwc,cv::Mat& WtC) const {
    if (!slideWindows_.empty() && poseUpdateFlg_) {
      slideWindows_.back()->getPoseInWorld(Rwc,WtC);
      return true;
    }
    return false;
  }

  /** \brief get all features coordinate in world 
   */ 
  std::vector<cv::Vec3f> getFeatsInWorld() const {
    return fsm_.getPointsInWorld();
  }

  void updateImuMeas(double t,const IMU & data) {
    std::lock_guard<std::mutex> imuLock(m_imu_);
    imuMeas_.addMeas(t,data);
    if (slideWindows_.size() > 5) {
      imuMeas_.clean(slideWindows_[0]->timestamp_);
    }
  }

  const CameraPtr getCameraPtr() const {
    return CameraPtr(cam_);
  }

  void buddleAdjustment();

 private:
  EstState state;
  Config* cfg_;
  Camera* cam_;
  Initializator* init_;
  FeatureTracker* feaTrcker_;
  FeatureManager fsm_;
  PnpSolver* pnpSolver_;
  std::vector<FramePtr> slideWindows_;
  std::mutex m_filter_,m_imu_;
  MeasurementTimeline<IMU> imuMeas_;
  PreIntegration* preInteNow_;
  Eigen::Matrix3d Rbc_;
  Eigen::Vector3d tbc_;
  bool poseUpdateFlg_;
  bool removeOldKeyFrame_;
  std::ofstream recordFile_;

  /** \brief slide window to remove oldest frame and features
   */ 
  void slideWindow();

  /** \brief estimate pose of current frame
   * @return return true if estimator works well otherwise return false
   */   
  bool estimatePose(FramePtr framePtr);

  /** \brief check pose ok or not
   * @return return true if current pose is ok
   */ 
  bool checkPose();

  /** \brief update features include removing untracked points and add new points
   */ 
  void updateFeature(FramePtr frame);

  void checkParallex(FramePtr frame);


  void calCameraRotationMatrix(double lastT, double curT, cv::Mat &R_cur_last);

};
}
