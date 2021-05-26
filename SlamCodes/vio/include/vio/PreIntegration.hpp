//
// Created by kdq on 2021/5/26.
//
//
#pragma once
#include "eigen3/Eigen/Dense"
#include "MeasurementTimeline.hpp"
class PreIntegration {
 public:
  PreIntegration(const Eigen::Vector3d & ba,const Eigen::Vector3d & bg) {
    dP_.setZero();
    dR_.setIdentity();
    dV_.setZero();
    ba_ = ba;
    bg_ = bg;
  };
  void progate(const IMU& imu) {
    if (imuLast_.timestamp_ < 0) {
      imuLast_ = imu;
    }
    double dt = imu.timestamp_ - imuLast_.timestamp_;
    Eigen::Vector3d accLast = imuLast_.acc_ - ba_;
    Eigen::Vector3d accNow = imu.acc_ - ba_;
    Eigen::Vector3d gyrLast = imuLast_.gyro_ - bg_;
    Eigen::Vector3d gyrNow = imu.gyro_ - bg_;
    Eigen::Matrix3d lastdR = dR_;
    Eigen::Vector3d dAngle = (gyrLast + gyrNow) * dt * 0.5;
    dR_ =
    //Eigen::Vector3d acc = (dR_ * imuLast_.acc
  }
 private:
  IMU imuLast_;
  Eigen::Vector3d dP_;
  Eigen::Matrix3d dR_;
  Eigen::Vector3d dV_;
  Eigen::Vector3d ba_;
  Eigen::Vector3d bg_;
};
