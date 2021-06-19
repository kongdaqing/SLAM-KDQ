#pragma once
#include <fstream>
#include "FeatureManager.hpp"
#include "Initializator.hpp"
#include "Config.hpp"
#include "PnpSolver.hpp"
#include "FeatureTracker.hpp"
#include "MeasurementTimeline.hpp"
#include "PreIntegration.hpp"
#include "BASolver.hpp"

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
   * @param configFile --- config file
   */ 
  Estimator(std::string configFile);
  
  /** \brief deconstruct function
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
   * @param Rwc --- rotation matrix from camera to world
   * @param WtC --- translation vector from camera to world
   * @return return pose success
   */ 
  bool getCurrentPose(cv::Mat& Rwc,cv::Mat& WtC) const {
    if (!slideWindows_.empty() && poseUpdateFlg_) {
      slideWindows_.back()->getPoseInWorld(Rwc,WtC);
      return true;
    }
    return false;
  }

  void getCurrentPose(Eigen::Vector3d &t,Eigen::Quaterniond& q) const;

  /** \brief get all features coordinate in world 
   */ 
  std::vector<cv::Vec3f> getFeatsInWorld() const {
    return fsm_.getPointsInWorld();
  }

  /** \brief update imu data
   *
   * @param t --- timestamp of imu data
   * @param data --- imu data
   */
  void updateImuMeas(double t,const IMU & data) {
    std::lock_guard<std::mutex> imuLock(m_imu_);
    imuMeas_.addMeas(t,data);
    if (!slideWindows_.empty()) {
      imuMeas_.clean(slideWindows_.back()->timestamp_ - 1.0);
    } else {
      while (imuMeas_.measMap_.size() > 1000) {
        imuMeas_.measMap_.erase(imuMeas_.measMap_.begin());
      }
    }
  }

  /** \brief get camera shared pointer
   * @return
   */
  const CameraPtr getCameraPtr() const {
    return CameraPtr(cam_);
  }

  /** \brief perform bundle adjustment using g2o solver
   *
   */
  void bundleAdjustment();

 private:
  EstState state;
  Config* cfg_;
  Camera* cam_;
  Initializator* init_;
  FeatureTracker* feaTrcker_;
  FeatureManager fsm_;
  PnpSolver* pnpSolver_;
  BAG2O * baSolver_;
  std::vector<FramePtr> slideWindows_;
  std::map<double,Eigen::Vector3d> slideAccel_,slidePose_;
  int goodExcitationCount_;
  std::mutex m_filter_,m_imu_;
  MeasurementTimeline<IMU> imuMeas_;
  PreIntegration* preInteNow_;
  Eigen::Matrix3d Rbc_;
  Eigen::Vector3d tbc_;
  bool poseUpdateFlg_;
  bool removeOldKeyFrame_;
  std::string moduleName_;
  const static int S = 20;
  const static int D = 3;
  const static int K = 5;
  std::ofstream splineFile;
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

  /** \brief check frame is keyframe or not through check parallax between frame and slidewindow back frame
   * @param frame --- frame
   */
  void isKeyframe(FramePtr frame);

  /** \brief calculate rotation matrix from last time to current time
   *
   * @param lastT --- last frame timestamp
   * @param curT --- current frame timestamp
   * @param R_cur_last --- rotation matrix from last frame to current frame
   */
  void calCameraRotationMatrix(double lastT, double curT, cv::Mat &R_cur_last);


   /** \brief update accel and pose in the slide window
    * @param curFrame --- current frame pointer
    */
  void updateSlideAccelAndPose(FramePtr curFrame);

  /** \brief get secord difference of pose thourgh b-spline
   *
   */
  void calWindowsAccelByBSpline();

  /** \brief calculate rotation matrix and scale from camera coordinate to world coordinate
   * @param splineAcc --- accel from b-spline
   * @param imuAcc --- accel from imu
   */
  void calScaleAndR0(const std::vector<Eigen::Vector3d>& splineAcc,const std::vector<Eigen::Vector3d>& imuAcc);

};
}
