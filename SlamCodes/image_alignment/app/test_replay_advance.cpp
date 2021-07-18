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
  parser.add<std::string>("camera_config", 'f', "camera config file.", false, "rovio_cheerios.yaml");//建议使用去畸变方式，否则计算的warp可能有零偏
  parser.add<int>("motion_model",'m',"image alignment model",false,1);
  parser.add<int>("iteration_number",'i',"iteration number of image alignment",false,25); //通常迭代十几次可能就收敛了，误差可能在1e-3以内且不会再下降，因此再迭代没有意义
  parser.add<double>("termination_eps",'e',"termination eps for ecc",false, 1e-3);//默认值已经够了，当迭代一定次数后，误差降低很不明显，因此不必要设置大的迭代次数
  parser.add<int>("pyramid_level",'l',"level of pyramid for image alignment",false,4);//金字塔等级越高，相应平移部分越小，L1和L4对比发现，平移部分呈现4倍关系，旋转部分L1波动要小一点
  parser.add<bool>("histogram_enable",'g',"enable equalize histogram flg",false ,false); //当前看对ecc估计效果影响不大
  parser.parse_check(argc,argv);
  std::string imageUrl;
  std::string cameraConfigFile;
  int alignModel = cv::MOTION_EUCLIDEAN;
  int iterationNum = 40;
  double terminationEps = 1e-3;
  int pyramidLevel = 1;
  bool histogramEnable = false;
  imageUrl = parser.get<std::string>("image_server_url");
  cameraConfigFile = parser.get<std::string>("camera_config");
  alignModel = parser.get<int>("motion_model");
  iterationNum = parser.get<int>("iteration_number");
  terminationEps = parser.get<double>("termination_eps");
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
  ImageAlignment imgAlign(cam,alignModel,iterationNum,terminationEps,pyramidLevel,histogramEnable);
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
  std::cout << "WarpMatrix:t,dt,cost,err,a00,a01,t0,a10,a11,t1,wx,wy" << std::endl;
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
    std::cout << warp1.at<float>(0,2) / imgAlign.getFocalLength() / dt << "," << warp1.at<float>(1,2) / imgAlign.getFocalLength() / dt << std::endl;
  }

  return 0;
}

