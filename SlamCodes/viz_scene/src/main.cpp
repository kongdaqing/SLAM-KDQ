#include "viz_scene/viz_scene.hpp"
#include "imu_motion/motion_para.hpp"
using namespace viz_scene;
int main(int argc,char** argv){
  if(argc == 2) {
    string configFile = argv[1];
    MotionParam para(configFile);
  }
  VizScene vizScene("test");
  vizScene.createRandomPlanePoints("test plane",Vec3f(2,2,2),Vec3f(1,1,1),100,1,1);
  while(1) {
    vizScene.testIncreasePoints("test plane");
    usleep(100);
  }
  return 0;
}