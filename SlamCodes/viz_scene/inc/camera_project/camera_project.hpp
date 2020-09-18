#pragma once
#include <fstream>
#include <random>
#include "camodocal/camera_models/CameraFactory.h"
#include "utilities/access_file.hpp"

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
    ProjectPointInfo projectVizPoints(double t,const cv::Mat& ptsCloud,const Eigen::Isometry3d& cameraPos);
    Eigen::Vector3d pixelInverseProjectToWorld(const Eigen::Isometry3d& cameraPos,Eigen::Vector2d uv,double scale);
    CameraPtr camPtr_;
};