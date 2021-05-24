//
// Created by kdq on 2021/5/24.
//
#include "iostream"
#include "rovio.pb.h"
#include "image.pb.h"
#include "replaykit.h"
#include "BottomImage.hpp"
#include "IMU.hpp"
#include "cmdline.h"

typedef ::zz::replaykit::ReplayKit<::zz::replaykit::Topics<vision::BottomImage, rovio::InputInfoPack>,::zz::replaykit::Commands<>> ReplayKitType;


int main(int argc,char **argv) {
  cmdline::parser parser;
  parser.add<std::string>("server_url", 'u', "Data server url.", false, "tcp://192.168.43.1");
  std::string serverBaseUrl;
  serverBaseUrl = parser.get<std::string>("server_url");
  ReplayKitType replaykit;
  nnstation::BottomClient bottomClient;
  nnstation::ImuClient imuClient;
  replaykit.Subscribe<0>([&](double now_time, const vision::BottomImage &bottomImage) {
    const cv::Mat im = cv::Mat(cv::Size(bottomImage.width(), bottomImage.height()), CV_8UC1,
                               (char *) bottomImage.image_buffer().c_str()).clone();
    auto delay = static_cast<float>(now_time - bottomImage.timestamp());
    auto exp_time = bottomImage.exposure_time();
    printf("[Image] Get %12.6f at %12.6f, exp = %7.3fms, delay = %7.3fms\n",
           bottomImage.timestamp(), now_time, exp_time * 1e3f, delay * 1e3f);
    cv::imshow("image",im);
    cv::waitKey(1);
    //rovioNode.imgCallback(bottomImage.timestamp() + exp_time * 0.5 + img_timestamp_offset, im, delay);
  });

  replaykit.Subscribe<1>([&](double now_time, const rovio::InputInfoPack &info_pack) {
    static double last_T = 0;
    for (size_t i = 0; i < info_pack.info_size(); i++) {
      std::cout << info_pack.info(i).t() << "," << info_pack.info(i).acc().x() << "," <<  info_pack.info(i).acc().y() << "," << info_pack.info(i).acc().z() << "," << std::endl;
    }
  });
  bottomClient.connect(serverBaseUrl + ":18950");
  bottomClient.subscribe([&](const vision::BottomImage &bottomImage) {replaykit.Publish<0>(replaykit.Now(), bottomImage);});
  imuClient.connect(serverBaseUrl + ":19051");
  imuClient.subscribe([&](const rovio::InputInfoPack &info_pack) {replaykit.Publish<1>(replaykit.Now(), info_pack);});
  bottomClient.startRecv();
  imuClient.startRecv();
  replaykit.Start();
  sleep(UINT32_MAX);
  return 0;
};