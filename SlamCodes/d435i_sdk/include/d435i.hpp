#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
struct D435IConfigParam {
  D435IConfigParam() {
    printf("[D435I-DefaultParameters]:default values are set according to realsense-viwer!\n");
    streamType = infrared;
    width = 640;
    height = 480;
    framerate = 30;
    auto_exposure = 1;
    exposure_time = 5000.0;
    exposure_gain = 80.;
    ae_point = 2000;
  }

  void print() {
    std::cout << "streamType = " << streamType << std::endl;
    std::cout << "width = " << width << std::endl;
    std::cout << "height = " << height << std::endl;
    std::cout << "framerate = " << framerate << std::endl;
    std::cout << "auto_exposure = " << auto_exposure << std::endl;
    std::cout << "exposure_time = " << exposure_time << std::endl;
    std::cout << "exposure_gain = " << exposure_gain << std::endl;
    std::cout << "ae_point = " << ae_point << std::endl;
  }
  enum Stream {
    infrared,
//    rgb,
//    depth,
  }; //当前仅支持双目
  int streamType;
  int width;
  int height;
  int framerate;
  int auto_exposure;
  float exposure_time; //us
  float exposure_gain;
  float ae_point; //this value work when auto-exposure is enabled,present mean intensity of image
};

class D435I {
 public:  
  D435I(){};

  D435I(const std::string &cfgFile);

  void start();


  bool getInfraredImages(double& timestamp,cv::Mat& leftImg,cv::Mat& rightImg) const;

 private:
  rs2::pipeline pipe_;
  D435IConfigParam stereoConfigParam_;
};
