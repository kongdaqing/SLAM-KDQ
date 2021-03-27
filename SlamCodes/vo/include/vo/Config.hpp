#pragma once
#include "opencv2/opencv.hpp"

namespace ov {
struct OptitrackParam {
  float minDist;
  float qualityLevel;
  int maxPoints; 
  int trackBack;
  double pyramidLevel;
  int iterations;
  double eps;
  void print() {
    std::cout << "========OptiTrack Parameters========" << std::endl;
    std::cout << "trackBack = " << trackBack << std::endl;
    std::cout << "qualityLevel = " << qualityLevel << std::endl;
    std::cout << "maxPoints = " << maxPoints << std::endl;
    std::cout << "minDist = " << minDist << std::endl;
    std::cout << "pyramidLevel = " << pyramidLevel << std::endl;
    std::cout << "iterations = " << iterations << std::endl;
    std::cout << "eps = " << eps << std::endl;
  }
};
struct CameraParam {
  cv::Mat K;
  cv::Mat D;
  int is_fisheye;
  int width,height;
  void print() {
    std::cout << "=========Camera Parameters=========" << std::endl;
    std::cout << "IsFisheye = " << is_fisheye << std::endl;
    std::cout << "Width = " << width << std::endl;
    std::cout << "Height = " << height << std::endl;
    std::cout << "Intrinisic Matrix : \n" << K << std::endl;
    std::cout << "Distortion Matrix : \n" << D << std::endl;
  }
};
struct InitialParam {
  float minDisparity;
  int minMatchedFeatNum;
  float reprojectErr;
  float homographyTransformErr;
  void print() {
    std::cout << "=========Initializator Parameters=========" << std::endl;
    std::cout << "minDisparity = " << minDisparity << std::endl;
    std::cout << "minMatchedFeatNum = " << minMatchedFeatNum << std::endl;
    std::cout << "reprojectErr = " << reprojectErr << std::endl;
    std::cout << "homographyTransformErr = " << homographyTransformErr << std::endl;
  }
};
class Config {
 public:

  /** \brief construct function
   */
  Config(std::string configFile){
    cv::FileStorage fs(configFile,cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "[Config]:Config file is not exist!" << std::endl;
      return;
    }
    optParam_.trackBack = fs["OptiTrack.TrackBack"];
    optParam_.maxPoints = fs["OptiTrack.MaxPoints"];
    optParam_.minDist = fs["OptiTrack.MinDist"];
    optParam_.qualityLevel = fs["OptiTrack.QualityLevel"];
    optParam_.pyramidLevel = fs["OptiTrack.PyramidLevel"];
    optParam_.iterations = fs["OptiTrack.Iterations"];
    optParam_.eps = fs["OptiTrack.EPS"];
    optParam_.print();

    fs["Camera.Intrinisic"] >> camParam_.K;
    fs["Camera.Distortion"] >> camParam_.D;
    camParam_.is_fisheye = fs["Camera.IsFisheye"]; 
    camParam_.width = fs["Camera.Width"];
    camParam_.height = fs["Camera.Height"];
    camParam_.print();

    iniParam_.minDisparity = fs["Init.MinDisparity"];
    iniParam_.minMatchedFeatNum = fs["Init.MinMatchedFeatNum"];
    iniParam_.reprojectErr = fs["Init.ReprojectErr"];
    iniParam_.homographyTransformErr = fs["Init.HomographyTransformErr"];
    iniParam_.print();
  };
  OptitrackParam optParam_;
  CameraParam camParam_;
  InitialParam iniParam_;
};

}