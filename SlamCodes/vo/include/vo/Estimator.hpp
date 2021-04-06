#pragma once 
#include "FeatureManager.hpp"
#include "Initializator.hpp"
#include "Config.hpp"
#include "PnpSolver.hpp"
#include "FeatureTracker.hpp"

namespace ov {
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
   * @param FramePtr  --- frame shared_ptr 
   */
  void update(FramePtr frame);

  /** \brief get estimator state
   */ 
  inline EstState getEstimatorState() const{
    return state;
  }
 private:
  EstState state;
  Config* cfg_;
  Camera* cam_;
  Initializator* init_;
  FeatureTracker* feaTrcker;
  FeatureManager fsm_;
  PnpSolver pnpSolver_;
  std::vector<FramePtr> slideWindows_;

  /** \brief slide window to remove oldest frame and features
   */ 
  void slideWindow();

  /** \brief estimate pose of current frame
   * @return return true if estimator works well otherwise return false
   */   
  bool estimatePose(); 

  /** \brief check pose ok or not
   * @return return true if current pose is ok
   */ 
  bool checkPose();

  /** \brief update features include removing untracked points and add new points
   */ 
  void updateFeature();

};
}