#pragma once
#include <fstream>
#include <random>
#include "camodocal/camera_models/CameraFactory.h"
using namespace camodocal;
class CameraProject {
  private:
    bool inBorder(const cv::Point2i& pt);
    std::string outPtsFile_;
    float pixelNoise_;
    bool addNoise_;
  public:
    CameraProject(bool addNoise,float pixelNoise,std::string cameraConfigFile,std::string outputPtsFile);
    ~CameraProject();
    void projectVizPoints(double t,const cv::Mat& ptsCloud,const Eigen::Isometry3d& cameraPos);
    CameraPtr camPtr_;
};