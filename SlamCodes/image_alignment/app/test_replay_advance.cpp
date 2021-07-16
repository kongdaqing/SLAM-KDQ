#include "iostream"
#include "nnstation/BottomImage.hpp"
#include "cmdline.h"
#include "ImageAlignment.hpp"
#include "ConfigLoad.hpp"
#include "map"
#include "mutex"
#include "condition_variable"


int main(int argc,char** argv) {
  cmdline::parser parser;
  parser.add<std::string>("image_server_url", 'u', "image server url.", false, "tcp://127.0.0.1:11900");
  parser.add<std::string>("camera_config", 'f', "camera config file.", false, "rovio_cheerios.yaml");
  parser.add<int>("motion_model",'m',"image alignment model",false,1);
  parser.add<int>("iteration_number",'i',"iteration number of image alignment",false,50);
  parser.add<int>("pyramid_level",'l',"level of pyramid for image alignment",false,1);
  parser.add<bool>("histogram_enable",'h',"enable equalize histogram flg",false ,false);
  parser.parse_check(argc,argv);
  std::string imageUrl;
  std::string cameraConfigFile;
  int alignModel = cv::MOTION_EUCLIDEAN;
  int iterationNum = 40;
  int pyramidLevel = 1;
  bool histogramEnable = false;
  imageUrl = parser.get<std::string>("image_server_url");
  cameraConfigFile = parser.get<std::string>("camera_config");
  alignModel = parser.get<int>("motion_model");
  iterationNum = parser.get<int>("iteration_number");
  pyramidLevel = parser.get<int>("pyramid_level");
  histogramEnable = parser.get<bool>("histogram_enable");
  if (iterationNum <= 0) {
    iterationNum = 10;
  }
  ConfigLoad cfg(cameraConfigFile);
  Camera* cam = nullptr;
  if (cfg.readOK_) {
    cam = new Camera(cfg.K_,cfg.D_,cfg.fisheye_,cfg.width_,cfg.height_);
  }
  ImageAlignment imgAlign(cam,alignModel,iterationNum,1e-10,pyramidLevel,histogramEnable);
  nnstation::BottomClient bottomClient;
  std::map<double,cv::Mat> imageBuffer;
  std::mutex imgLck;
  std::condition_variable imgCv;
  bottomClient.connect(imageUrl);
  bottomClient.subscribe([&](const vision::BottomImage &bottomImage) {
    cv::Mat im = cv::Mat(cv::Size(bottomImage.width(), bottomImage.height()), CV_8UC1,
                         (char *) bottomImage.image_buffer().c_str()).clone();
    imgLck.lock();
    imageBuffer[bottomImage.timestamp()] = im;
    imgLck.unlock();
    imgCv.notify_all();
  });
  bottomClient.startRecv();
  double last_t = 0;
  std::cout << "WarpMatrix:t,dt,cost,err,a00,a01,t0,a10,a11,t1," << std::endl;
  while(1) {
    std::unique_lock<std::mutex> lck(imgLck);
    imgCv.wait(lck,[&](){return !imageBuffer.empty();});
    double t = imageBuffer.begin()->first;
    cv::Mat img = imageBuffer.begin()->second.clone();
    imageBuffer.erase(imageBuffer.begin());
    double err;
    const double toc_start  = (double) cv::getTickCount ();
    cv::Mat warp1 = imgAlign.Align(t,img,err);
    const double toc_final  = ((double) cv::getTickCount () - toc_start) / cv::getTickFrequency();
    double dt = t - last_t;
    last_t = t;
    if (dt > 10.) {
      continue;
    }
    std::cout << "WarpMatrix:" << t << ","  << dt << "," << toc_final << "," << err << ",";
    for (size_t i = 0; i < warp1.rows; i++) {
      for (size_t j = 0; j < warp1.cols; j++) {
        std::cout << warp1.at<float>(i,j) << ",";
      }
    }
    std::cout << std::endl;
  }

  return 0;
}

