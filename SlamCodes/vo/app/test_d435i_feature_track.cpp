#include "d435i.hpp"
#include "Camera.hpp"
#include "FeatureTracker.hpp"
using namespace ov;



void showAllFeature(const std::list<std::map<uint64,cv::Point2f>> &corners,cv::Mat &img,uint scale) {
  cv::Mat colorImg;
  cv::cvtColor(img,colorImg,cv::COLOR_GRAY2BGR);
   
  std::map<uint64,cv::Point2f> cornerFront = corners.front();
  std::map<uint64,cv::Point2f> cornerBack = corners.back();
  for(auto it = cornerBack.begin(); it != cornerBack.end(); it++) {
    uint64 id = it->first;
    if (cornerFront.count(id)) {
      cv::Point2f pointBegin = cornerFront[id];
      cv::Point2f pointEnd = cornerBack[id];
      cv::line(colorImg,pointBegin,pointEnd,cv::Scalar(0,255,0));
    }
  }
  cv::resize(colorImg,colorImg,cv::Size(colorImg.cols * scale, colorImg.rows * scale));
  cv::imshow("track test",colorImg);
  cv::waitKey(1);
}

int main(int argc,char **argv) {
  if (argc < 2) {
    std::cerr << "please input config file!" << std::endl;
    return -1;
  }
  std::string cfgFile = argv[1];
  Config* cfg = new Config(cfgFile);
  Camera* cam = new Camera(cfg);
  FeatureTracker *featTracker = new FeatureTracker(cfg);
  FramePtr lastF = nullptr;
  D435I d435i_device;
  d435i_device.start();
  std::list<std::map<uint64,cv::Point2f> > windowsFeatures;
  double time_s = 0;
  while (1) {
    time_s++;
    cv::Mat leftImg,rightImg;
    d435i_device.getInfraredImages(leftImg,rightImg);
    if (!leftImg.empty()) {
      FramePtr curF(new Frame(time_s,leftImg));
      featTracker->detectAndTrackFeature(lastF,curF);
      lastF = curF;
      curF->imshowFeatures(2);
      std::map<uint64,cv::Point2f> curCorners = curF->getCornersCopy();
      windowsFeatures.push_back(curCorners);
      showAllFeature(windowsFeatures,curF->image_,1);
      if (windowsFeatures.size() > 5) {
       windowsFeatures.pop_front();
      }
    }
    cv::waitKey(10);
  }
}
