#include <iostream>
#include <opencv2/viz.hpp>
#include <eigen3/Eigen/Dense>
#include "viz_scene/viz_scene.hpp"
using namespace viz_scene;
int main(int argc,char** argv) {
  cv::Mat a;
  VizScene testViz("windows");
  Eigen::Vector2f ad;
  while(1) {
    usleep(10);
  }
  printf("Hello VO!\n");
}
