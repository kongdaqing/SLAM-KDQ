#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

class D435I {
 public:  
  D435I() {
    cfg_.disable_stream(RS2_STREAM_DEPTH);
    cfg_.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg_.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
  }

  void start();


  void getInfraredImages(cv::Mat& leftImg,cv::Mat& rightImg);

 private:
  rs2::pipeline pipe_;
  rs2::config cfg_;
};
