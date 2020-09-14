#include "viz_scene/viz_scene.hpp"
#include "imu_motion/imu_motion.hpp"
#include "utilities/access_file.hpp"
using namespace viz_scene;
int main(int argc,char** argv){
  if(argc < 2) {
    cout << "please input config file!" << endl; 
    return -1;
  }
  string configFile = argv[1];
  MotionParam* para = new MotionParam(configFile);
  ImuMotion imuModel(para);
  VizScene vizScene("test");
  vizScene.createRandomPlanePoints("test plane",Vec3f(0,0,0),Vec3f(0,0,1),100,2,2,0.5);
  vizScene.createCameraObject("test camera",0.3,Vec2f(0.3,0.4),Vec3f(0,0,0.2),Vec3f(0,0,0.1),Vec3f(0,1.0,0));
  double t = para->start_t_;
  double dt = 1.0 / (double)(para->imuFreq_);
  VioDatasInterface::recordImuMotionState(ImuMotionState(),"test.csv",true);
  VioDatasInterface::recordImuMotionState(ImuMotionState(),"noise_test.csv",true);
  VioDatasInterface::recordPoseAsTum(ImuMotionState(),"real_pose.csv",true);
  while(1) {
    //vizScene.testIncreasePoints("test plane");
    if(t  < para->end_t_) {
      ImuMotionState imuState = imuModel.simImuMotion(t);
      ImuMotionState imuNoiseState = imuModel.addImuNoise(imuState);
      VioDatasInterface::recordImuMotionState(imuState,"test.csv");
      VioDatasInterface::recordPoseAsTum(imuState,"real_pose.csv");
      VioDatasInterface::recordImuMotionState(imuNoiseState,"noise_test.csv");
      t += dt; 
      Eigen::Matrix3d Rwb(imuState.qwi_);
      Eigen::Matrix3d Rwc = Rwb * para->Rbc_;
      Eigen::Vector3d twc = Rwb * para->tbc_ + imuState.pos_;
      Mat3d Rwc_cv;
      eigen2cv(Rwc,Rwc_cv);
      Vec3d twc_cv(twc.x(),twc.y(),twc.z());
      Affine3d Twc_cv(Rwc_cv,twc_cv);
      vizScene.updateCameraPose("test camera",Twc_cv);
    } else {
      std::vector<ImuMotionState> imuData;
      VioDatasInterface::readImuMotionState(imuData,"test.csv");
      std::cout << "imu data size = " << imuData.size() << std::endl;
      (imuData.end()-1)->printData();
      imuModel.testImuMotionData("test.csv","pose.csv");
      imuModel.testImuMotionData("noise_test.csv","noise_pose.csv");
      break;
    }
    usleep(100);
  }
  return 0;
}