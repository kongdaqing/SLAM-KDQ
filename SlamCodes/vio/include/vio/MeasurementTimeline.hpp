//
// Created by kdq on 2021/5/26.
//
#pragma once
#include "eigen3/Eigen/Eigen"

struct IMU {
  IMU() {
    timestamp_ = -0.001;
    acc_.setZero();
    gyro_.setZero();
  }
  void operator = (const IMU& imu) {
    timestamp_ = imu.timestamp_;
    acc_ = imu.acc_;
    gyro_ = imu.gyro_;
  }

  IMU(double t,const Eigen::Vector3d& acc,const Eigen::Vector3d& gyro) {
    timestamp_ = t;
    acc_ = acc;
    gyro_ = gyro;
  }
  double timestamp_;
  Eigen::Vector3d acc_;
  Eigen::Vector3d gyro_;
};

template<typename Meas>
class MeasurementTimeline {
 public:
  MeasurementTimeline() {
  }
  void clear() {
    measMap_.clear();
  };
  void addMeas(double t,const Meas& meas) {
    measMap_[t] = meas;
  }
  void clean(double t) {
    while (!measMap_.empty() && measMap_.begin()->first <= t) {
      measMap_.erase(measMap_.begin());
    }
  }
  bool empty() {
    return measMap_.empty();
  }
 private:
  std::map<double,Meas> measMap_;
};
