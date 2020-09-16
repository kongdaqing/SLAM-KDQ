#pragma once
#include <iostream>
#include <map>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "imu_motion/imu_motion.hpp"

class VioDatasInterface
{
private:
  
public:
  VioDatasInterface(/* args */);
  ~VioDatasInterface();

  static void recordImuMotionState(const ImuMotionState& data,const std::string fileName,bool title = false) {
    if(title) {
      std::fstream file(fileName,std::ios::out);
      file << "t,px,py,pz,qw,qx,qy,qz,vx,vy,vz,ax,ay,az,wx,wy,wz,bax,bay,baz,bwx,bwy,bwz\n";
      return;
    }
    std::fstream file(fileName,std::ios::app);
    file << data.timestamp_ << " "
         << data.pos_.x() << " " 
         << data.pos_.y() << " " 
         << data.pos_.z() << " " 
         << data.qwi_.w() << " "
         << data.qwi_.x() << " "
         << data.qwi_.y() << " "
         << data.qwi_.z() << " "
         << data.vel_.x() << " "
         << data.vel_.y() << " "
         << data.vel_.z() << " "
         << data.acc_.x() << " "
         << data.acc_.y() << " "
         << data.acc_.z() << " "
         << data.gyr_.x() << " "
         << data.gyr_.y() << " "
         << data.gyr_.z() << " "
         << data.acc_bias_.x() << " "
         << data.acc_bias_.y() << " "
         << data.acc_bias_.z() << " "
         << data.gyr_bias_.x() << " "
         << data.gyr_bias_.y() << " "
         << data.gyr_bias_.z() << "\n";
  }

  static void readImuMotionState(std::vector<ImuMotionState>& dataVec,const std::string fileName) {
    std::fstream file(fileName,std::ios::in);

    if (file.is_open()) {
      std::string tmp;
      std::getline(file,tmp);
      while(!file.eof()) {
        ImuMotionState data;
        file >> data.timestamp_;
        file >> data.pos_[0]; 
        file >> data.pos_[1]; 
        file >> data.pos_[2];
        file >> data.qwi_.w();
        file >> data.qwi_.x();
        file >> data.qwi_.y();
        file >> data.qwi_.z();
        file >> data.vel_[0]; 
        file >> data.vel_[1]; 
        file >> data.vel_[2];  
        file >> data.acc_[0]; 
        file >> data.acc_[1]; 
        file >> data.acc_[2];  
        file >> data.gyr_[0]; 
        file >> data.gyr_[1]; 
        file >> data.gyr_[2];  
        file >> data.acc_bias_[0]; 
        file >> data.acc_bias_[1]; 
        file >> data.acc_bias_[2];  
        file >> data.gyr_bias_[0]; 
        file >> data.gyr_bias_[1]; 
        file >> data.gyr_bias_[2];
        dataVec.push_back(data);
      }
      dataVec.erase(dataVec.end()-1);
    } else {
      std::cout << fileName << " can't be open" << std::endl;
    }
  }
  static void recordPoseAsTum(const ImuMotionState& data,const std::string fileName,bool title = false) {
    if(title) {
      std::fstream file(fileName,std::ios::out);
      file << "t px py pz qx qy qz qw\n";
      return;
    }
    std::fstream file(fileName,std::ios::app);
        file.precision(9);
        file << data.timestamp_ << " ";
        file.precision(5);
        file << data.pos_.x() << " "
             << data.pos_.y() << " "
             << data.pos_.z() << " "
             << data.qwi_.x() << " "
             << data.qwi_.y() << " "
             << data.qwi_.z() << " "
             << data.qwi_.w() <<std::endl;
  }
  static void recordCameraPixel(double t,const std::map<int,std::pair<cv::Point3d,cv::Point2i>>& ptsMap,const std::string fileName,bool title = false) {
    if (title) {
      std::fstream file(fileName,std::ios::out);
      file << "t,id p1W_x p1W_y p1W_z pix1_x pix1_y, ...\n"; 
      return;
    }
    std::fstream file(fileName,std::ios::app);
    file.precision(9);
    file << t << ",";
    std::map<int,std::pair<cv::Point3d,cv::Point2i> >::const_iterator it;
    for (it = ptsMap.begin();it != ptsMap.end();it++) {
      std::pair<cv::Point3d,cv::Point2i> pt = it->second;
      file << it->first << " " << pt.first.x << " " << pt.first.y << " " << pt.first.z << " " << pt.second.x << " " << pt.second.y << ","; 
    }
    file << "\n";
  }

};
