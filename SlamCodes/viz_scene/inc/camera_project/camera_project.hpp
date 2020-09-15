#pragma once
#include <fstream>
#include "camodocal/camera_models/CameraFactory.h"
using namespace camodocal;
class CameraProject {
  private:
    bool inBorder(const Eigen::Vector2d& pt);
    std::string outPtsFile;
  public:
    CameraProject(std::string cameraConfigFile,std::string outputPtsFile);
    ~CameraProject();
    void projectVizPoints(double t,const cv::Mat& ptsCloud,const Eigen::Isometry3d& cameraPos);
    CameraPtr camPtr_;
};