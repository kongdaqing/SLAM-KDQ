#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "utilities/kinematic.hpp"
#include "imu_motion/motion_para.hpp"

struct ImuMotionState {
  double timestamp_;
  Eigen::Vector3d pos_;
  Eigen::Quaterniond qwi_;
  Eigen::Vector3d vel_; // velocity of imu in the world frame
  Eigen::Vector3d acc_;
  Eigen::Vector3d gyr_;
  Eigen::Vector3d acc_bias_;
  Eigen::Vector3d gyr_bias_;
};


class ImuMotion {
  private:
    ImuMotionState init_state_;
    const MotionParam* para_;
  public:
    ImuMotion(const MotionParam* motionParam):para_(motionParam){};
    ~ImuMotion();
    ImuMotionState simImuMotion(double t);
    void addImuNoise(ImuMotionState& data);
    void testImuMotionData();
};