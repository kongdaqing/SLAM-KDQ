#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <opencv/cv.hpp>
#include <opencv2/core/eigen.hpp>
class MotionParam {
  private:
  public:
    MotionParam(std::string configFile) {
      cv::FileStorage config(configFile,cv::FileStorage::READ);
      imuFreq_ = config["imuFreq"];
      imageFreq_ = config["imageFreq"];
      start_t_ = config["simStart"];
      end_t_ = config["simEnd"];
      gyr_noise_sigma_ = config["gyrNoise"];
      acc_noise_sigma_ = config["accNoise"];
      gyr_bias_sigma_ = config["gyrBiasNoise"];
      acc_bias_sigma_ = config["accBiasNoise"];
      pixel_noise_ = config["pixelNoise"];
      cv::Mat Tbc,tbc;
      Eigen::Matrix4d Tbc_tmp;
      config["Tbc"] >> Tbc;
      cv::cv2eigen(Tbc,Tbc_tmp);
      Rbc_ = Tbc_tmp.block<3,3>(0,0);
      tbc_ = Tbc_tmp.block<3,1>(0,3);     
      std::cout << "imuFreq = " << imuFreq_ << std::endl;
      std::cout << "imageFreq = " << imageFreq_ << std::endl;
      std::cout << "Rbc = \n" << Rbc_ << std::endl;
      std::cout << "tbc = " << tbc_.transpose() << std::endl; 
    };
    ~MotionParam(){};
    int imuFreq_,imageFreq_;
    double start_t_,end_t_;
    double gyr_noise_sigma_,acc_noise_sigma_;
    double gyr_bias_sigma_,acc_bias_sigma_;
    double pixel_noise_;

    Eigen::Matrix3d Rbc_;
    Eigen::Vector3d tbc_;
};