#include "d435i.hpp"

D435I::D435I(const std::string &cfgFile) {
  cv::FileStorage fs(cfgFile,cv::FileStorage::Mode::READ);
  if (!fs.isOpened()) {
    printf("[D435I]:config file open failed and d435i will initialize with default parameters!\n");
  } else {
    stereoConfigParam_.streamType = fs["stereo.stream"];
    stereoConfigParam_.width = fs["stereo.width"];
    stereoConfigParam_.height = fs["stereo.height"];
    stereoConfigParam_.auto_exposure = fs["stereo.auto_exposure"];
    stereoConfigParam_.framerate = fs["stereo.frame_rate"];
    stereoConfigParam_.exposure_time = fs["stereo.exposure_time"];
    stereoConfigParam_.exposure_gain = fs["stereo.exposure_gain"];
    stereoConfigParam_.ae_point = fs["stereo.ae_point"];
    stereoConfigParam_.print();
  }
}

void D435I::start() {
  rs2::config cfg;
  if (stereoConfigParam_.streamType == D435IConfigParam::Stream::infrared) {
    cfg.disable_all_streams();
    cfg.enable_stream(RS2_STREAM_INFRARED, 1,
                      stereoConfigParam_.width, stereoConfigParam_.height,
                      RS2_FORMAT_Y8, stereoConfigParam_.framerate);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2,
                      stereoConfigParam_.width, stereoConfigParam_.height, RS2_FORMAT_Y8,
                      stereoConfigParam_.framerate);

  }
  rs2::pipeline_profile selection = pipe_.start(cfg);
  rs2::device selected_device = selection.get_device();
  auto depth_sensor = selected_device.first<rs2::depth_sensor>();
  if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
  }
  if (stereoConfigParam_.auto_exposure) {
    depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,1);
    auto advMode = rs400::advanced_mode(selected_device);
    auto aeCtrl = advMode.get_ae_control();
    aeCtrl.meanIntensitySetPoint = stereoConfigParam_.ae_point;
    advMode.set_ae_control(aeCtrl);
  } else {
    depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,0);
    depth_sensor.set_option(RS2_OPTION_EXPOSURE,stereoConfigParam_.exposure_time);
    depth_sensor.set_option(RS2_OPTION_GAIN,stereoConfigParam_.exposure_gain);
  }
  for (size_t i = 0; i < 10;) {
    cv::Mat leftImg,rightImg;
    double timestamp;
    if (getInfraredImages(timestamp,leftImg,rightImg)) {
      i++;
    }
  }
}

bool D435I::getInfraredImages(double& timestamp,cv::Mat& leftImg,cv::Mat& rightImg) const{
  rs2::frameset data = pipe_.wait_for_frames();
  if (data) {
    rs2::video_frame infrared1 = data.get_infrared_frame(1);
    rs2::video_frame infrared2 = data.get_infrared_frame(2);
    timestamp = infrared1.get_timestamp() * 1e-3;
    std::cout << std::setprecision(13) << "timestamp: " << timestamp << std::endl;
    // double exp_time = data.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
    // double frame_time = data.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP) * 1e-6;
    // std::cout << "frame_time  = " << frame_time <<  "  Real Stamp: " <<  exp_time << std::endl;
    cv::Mat infraredImg1(infrared1.get_height(),infrared1.get_width(),CV_8UC1,(void *)infrared1.get_data());
    infraredImg1.copyTo(leftImg);
    cv::Mat infraredImg2(infrared2.get_height(),infrared2.get_width(),CV_8UC1,(void *)infrared2.get_data());
    infraredImg2.copyTo(rightImg);
    return true;
  }
  return false;
}