#include "imu_motion/imu_motion.hpp"

ImuMotionState ImuMotion::simImuMotion(double t) {
  ImuMotionState stateNow;
  stateNow.timestamp_ = t;
  Eigen::Vector3d acc;
  //step1 : calculate pos and velocity,accelerate in world frame
  for (size_t i = 0; i < 3; i++)
  {
    //Pos Model = k * sin(a*t) + b
    stateNow.pos_[i] = para_->Skew_[0][i] * sin(para_->Phase_[0][i] * t) + para_->Bias_[0][i];
    stateNow.vel_[i] = para_->Skew_[0][i] * cos(para_->Phase_[0][i] * t);
    acc[i] = -para_->Skew_[0][i] * sin(para_->Phase_[0][i] * t);
  }
  //step 2 : calculate euler angle and rate 
  Eigen::Vector3d eulerAngleDeg,eulerRateDeg;
  for (size_t i = 0; i < 3; i++)
  {
    //Att Model = k * sin(a*t) + b
    eulerAngleDeg[i] = para_->Skew_[1][i] * sin(para_->Phase_[1][i] * t) + para_->Bias_[1][i];
    eulerRateDeg[i] = para_->Skew_[1][i] * cos(para_->Phase_[1][i] * t);
  }
  Eigen::Vector3d eulerAngleRad = eulerAngleDeg * M_PI / 180.0;
  Eigen::Vector3d eulerRateRad = eulerRateDeg * M_PI / 180.0;
  //step 3 : get Rotation quanternion
  Eigen::Matrix3d Rwb;
  kinetic::euler2RotationMatrix(eulerAngleRad,Rwb);
  stateNow.qwi_ = Eigen::Quaterniond(Rwb).normalized();
  //step 4 : get gyro data
  Eigen::Matrix3d R_euler2body;
  kinetic::eulerRate2BodyRate(eulerAngleRad,R_euler2body);
  stateNow.gyr_ = R_euler2body * eulerRateRad;
  //step 5: get acc data
  Eigen::Vector3d g(0.,0.,-9.81);
  stateNow.acc_ = Rwb.transpose() * (acc - g);
  return stateNow;
}