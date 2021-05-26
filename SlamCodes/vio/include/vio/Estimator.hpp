#pragma once 
#include "FeatureManager.hpp"
#include "Initializator.hpp"
#include "Config.hpp"
#include "PnpSolver.hpp"
#include "FeatureTracker.hpp"

namespace vio{
enum EstState {
  Waiting,
  Initing,
  Runing
};

class Estimator {
 public:
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
  bool getCurrentPose(cv::Mat& Rwc,cv::Mat& WtC) {
    if (!slideWindows_.empty()) {
      slideWindows_.back()->getPoseInWorld(Rwc,WtC);
      return true;
    }
    return false;
  }

  /** \brief get all features coordinate in world 
   */ 
  std::vector<cv::Vec3f> getFeatsInWorld() {
    return fsm_.getPointsInWorld();
  }
 private:
  EstState state;
  Config* cfg_;
  Camera* cam_;
  Initializator* init_;
  FeatureTracker* feaTrcker_;
  FeatureManager fsm_;
  PnpSolver* pnpSolver_;
  std::vector<FramePtr> slideWindows_;
  std::mutex m_filter_;


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
  void updateFeature();

};
}
