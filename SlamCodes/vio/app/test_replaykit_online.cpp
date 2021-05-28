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
#include "Estimator.hpp"

using namespace vio;
typedef ::zz::replaykit::ReplayKit<::zz::replaykit::Topics<vision::BottomImage, rovio::InputInfoPack>,::zz::replaykit::Commands<>> ReplayKitType;


int main(int argc,char **argv) {
  cmdline::parser parser;
  parser.add<std::string>("server_url", 'u', "Data server url.", false, "tcp://192.168.43.1");
  parser.add<std::string>("config_file",'c',"estimator config file",false,"../config/default.yaml");
  parser.add<int>("div_freq",'f',"divide freqency(hz)", false,1);
  parser.add<double>("time_offset",'o',"image time offset",false,0);
  parser.parse_check(argc, argv);
  std::string serverBaseUrl;
  std::string configFile;
  int divideFreq = 1;
  double timeoffset = 0.;
  serverBaseUrl = parser.get<std::string>("server_url");
  configFile = parser.get<std::string>("config_file");
  divideFreq = parser.get<int>("div_freq");
  timeoffset = parser.get<double>("time_offset");
  Estimator estimator(configFile);

  ReplayKitType replaykit;
  nnstation::BottomClient bottomClient;
  nnstation::ImuClient imuClient;
  int imageCount = 0;

  replaykit.Subscribe<0>([&](double now_time, const vision::BottomImage &bottomImage) {
    cv::Mat im = cv::Mat(cv::Size(bottomImage.width(), bottomImage.height()), CV_8UC1,
                               (char *) bottomImage.image_buffer().c_str()).clone();
    auto delay = static_cast<float>(now_time - bottomImage.timestamp());
    auto exp_time = bottomImage.exposure_time();
    imageCount++;
    if (imageCount == divideFreq) {
      imageCount = 0;
//      printf("[Image] Get %12.6f at %12.6f, exp = %7.3fms, delay = %7.3fms\n",
//             bottomImage.timestamp(), now_time, exp_time * 1e3f, delay * 1e3f);
      vio::FramePtr frame(new Frame(bottomImage.timestamp() + exp_time * 0.5 + timeoffset, im));
      estimator.update(frame, true);
    }
  });

  replaykit.Subscribe<1>([&](double now_time, const rovio::InputInfoPack &info_pack) {
    static double last_T = 0;
    for (size_t i = 0; i < info_pack.info_size(); i++) {
      const rovio::InputInfo info = info_pack.info(i);
      double timestamp = info.t();
      Eigen::Vector3d acc(info.acc().x(),info.acc().y(),info.acc().z());
      Eigen::Vector3d gyr(info.gyr().x(),info.gyr().y(),info.gyr().z());
//      std::cout << "[IMU]: " << timestamp << ",ax:" << acc.x() << ",ay:" << acc.y() << ",az:" << acc.z() << std::endl;
//      std::cout << "[IMU]: " << timestamp << ",gx:" << gyr.x() * 60 << ",gy:" << gyr.y() * 60 << ",gz:" << gyr.z() * 60 << std::endl;
      estimator.updateImuMeas(timestamp,IMU(timestamp,acc,gyr));
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