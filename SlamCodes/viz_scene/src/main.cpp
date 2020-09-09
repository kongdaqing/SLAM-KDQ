#include "viz_scene/viz_scene.hpp"
using namespace viz_scene;
int main(){
  VizScene vizScene("test");
  vizScene.createRandomPlanePoints("test plane",Vec3f(2,2,2),Vec3f(1,1,1),100,1,1);
  while(1) {
    vizScene.testIncreasePoints("test plane");
    usleep(100);
  }
  return 0;
}