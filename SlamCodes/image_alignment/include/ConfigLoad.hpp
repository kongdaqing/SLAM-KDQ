//
// Created by kdq on 2021/6/29.
//
#pragma once
#include "iostream"
#include "fstream"
#include "opencv2/opencv.hpp"


class ConfigLoad {
 public:
  ConfigLoad() = default;

  ConfigLoad(std::string camConfigFile) {
    readOK_ = false;
    std::ifstream camFile(camConfigFile,std::ios::in);
    if (!camFile.good()) {
      printf("Read camera config file failure!");
      return;
    }
    while (!camFile.eof()) {
      std::string line;
      getline(camFile,line);
      std::stringstream str1(line);
      std::string s1,s2;
      str1 >> s1 >> s2;

      if (s1 == "image_width:") {
        std::stringstream str2(s2);
        str2 >> width_;
      } else if (s1 == "image_height:") {
        std::stringstream str2(s2);
        str2 >> height_;
      } else if (s1 == "distortion_model:") {
        fisheye_ = s2 == "equidistant" ? true : false;
      } else if (s1 == "camera_matrix:") {
        std::string tmp;
        getline(camFile,tmp);
        getline(camFile,tmp);
        tmp.clear();
        getline(camFile,tmp);
        std::stringstream strTemp(tmp);
        std::string title;
        float fx,fy,ux,uy,d1,d2;
        char midH,dou1,dou2,dou3,dou4,dou5;
        strTemp >> title >> midH >> fx >> dou1 >> d1  >> dou2 >> ux >> dou3 >> d2 >> dou4 >> fy >> dou5 >> uy;
        K_ = (cv::Mat_<float>(3,3) << fx,0,ux,0,fy,uy,0,0,1);
      } else if (s1 == "distortion_coefficients:") {
        std::string tmp;
        getline(camFile,tmp);
        getline(camFile,tmp);
        tmp.clear();
        getline(camFile,tmp);
        std::stringstream strTemp(tmp);
        std::string title;
        float k1,k2,k3,k4;
        char midH,dou1,dou2,dou3;
        strTemp >> title >> midH >> k1 >> dou1 >> k2  >> dou2 >> k3 >> dou3 >> k4;
        D_ = (cv::Mat_<float>(1,4) << k1,k2,k3,k4);
      }
    }
    if (width_ > 0 && width_ < 10000 && height_ > 0 && height_ < 10000 &&
        K_.at<float>(0,0) > 0 && K_.at<float>(0,0) < 10000 &&
        K_.at<float>(0,2) > 0 && K_.at<float>(0,2) < 10000) {
      readOK_ = true;
    }
  }

  cv::Mat K_;
  cv::Mat D_;
  bool fisheye_;
  int width_;
  int height_;
  bool readOK_;
 private:

};