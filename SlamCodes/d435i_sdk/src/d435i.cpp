#include "d435i.hpp"

void D435I::start() {
  rs2::pipeline_profile selection = pipe_.start(cfg_);
  rs2::device selected_device = selection.get_device();
  auto depth_sensor = selected_device.first<rs2::depth_sensor>();
  if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
  }
}

void D435I::getInfraredImages(cv::Mat& leftImg,cv::Mat& rightImg) {
  rs2::frameset data = pipe_.wait_for_frames();
  if (data) {
    rs2::video_frame infrared1 = data.get_infrared_frame(1);
    rs2::video_frame infrared2 = data.get_infrared_frame(2);
    cv::Mat infraredImg1(infrared1.get_height(),infrared1.get_width(),CV_8UC1,(void *)infrared1.get_data());
    infraredImg1.copyTo(leftImg);
    cv::Mat infraredImg2(infrared2.get_height(),infrared2.get_width(),CV_8UC1,(void *)(infrared2.get_data()));
    infraredImg2.copyTo(rightImg);
  }
}