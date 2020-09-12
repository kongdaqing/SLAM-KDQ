#include "imu_motion/imu_motion.hpp"

ImuMotionState ImuMotion::addImuNoise(const ImuMotionState& data) {
  std::random_device rd;
  std::default_random_engine rdEngine(rd());
  std::normal_distribution<double> noise(0.0,1.0);
  ImuMotionState outData;

  Eigen::Vector3d gyro_noise(noise(rdEngine),noise(rdEngine),noise(rdEngine));
  Eigen::Matrix3d gyro_sqrt_cov  = para_->gyr_noise_sigma_ * Eigen::Matrix3d::Identity();
  outData.gyr_ = data.gyr_ + gyro_sqrt_cov * gyro_noise / sqrt(para_->imuInterval_) + gyr_bias_;
  
  Eigen::Vector3d acc_noise(noise(rdEngine),noise(rdEngine),noise(rdEngine));
  Eigen::Matrix3d acc_sqrt_cov  = para_->acc_noise_sigma_ * Eigen::Matrix3d::Identity();
  outData.acc_ = data.acc_ + acc_sqrt_cov * acc_noise / sqrt(para_->imuInterval_) + acc_bias_;

  Eigen::Vector3d gyro_bias_noise(noise(rdEngine),noise(rdEngine),noise(rdEngine));
  gyr_bias_ += para_->gyr_bias_sigma_ * sqrt(para_->imuInterval_) * gyro_bias_noise;
  outData.gyr_bias_ = gyr_bias_;

  Eigen::Vector3d acc_bias_noise(noise(rdEngine),noise(rdEngine),noise(rdEngine));
  acc_bias_ += para_->acc_bias_sigma_ * sqrt(para_->imuInterval_) * acc_bias_noise;
  outData.acc_bias_ = acc_bias_;
}


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

  if(t == 0) {
    init_state_ = stateNow;
  }
  return stateNow;
}



