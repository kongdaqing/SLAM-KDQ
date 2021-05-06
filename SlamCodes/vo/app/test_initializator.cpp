#include <random>
#include "Camera.hpp"
#include "Initializator.hpp"
#include "Frame.hpp"
using namespace std;
using namespace ov;
int main(int argc,char **argv) {
  if (argc < 2) {
    std::cerr << "please input config file" << std::endl;
  }
  string cfgFile = argv[1];
  Config* cfg = new Config(cfgFile);
  Camera* cam = new Camera(cfg);
  Initializator ini(cfg,cam);
  //Initializing frames
  Frame *refFrame = new Frame();
  Frame *curFrame = new Frame();
  cv::Mat rwc0 = (cv::Mat_<double>(3,1) << 0.,0.,0.);
  cv::Mat WtC0 = (cv::Mat_<double>(3,1) << 0.,0.,0.);
  cv::Mat Rwc0;
  cv::Rodrigues(rwc0,Rwc0);
  refFrame->setPoseInWorld(Rwc0,WtC0);
  //Note:注意如果设置的角度看不到特征点,会死在循环里
  cv::Mat rwc1 = (cv::Mat_<double>(3,1) << 0.,0.,0.);
  cv::Mat WtC1 = (cv::Mat_<double>(3,1) << 0.2,0.25,0.1);
  cv::Mat Rwc1;
  cv::Rodrigues(rwc1,Rwc1);
  curFrame->setPoseInWorld(Rwc1,WtC1);
  //Set pts in world
  random_device rd;
  mt19937 gen(rd());
  normal_distribution<float> pt3Dx(0,2);//设置均值和标准差
  normal_distribution<float> pt3Dy(0,2);
  normal_distribution<float> pt3Dz(1,0.01);
  cv::Mat Rcw0,CtW0,rcw0,Rcw1,CtW1,rcw1;
  refFrame->getInversePose(Rcw0,CtW0);
  cv::Rodrigues(Rcw0,rcw0);
  curFrame->getInversePose(Rcw1,CtW1);
  cv::Rodrigues(Rcw1,rcw1);
  std::map<uint64,cv::Point2f> &refCorners = refFrame->getCorners();
  std::map<uint64,cv::Point2f> &curCorners = curFrame->getCorners();
  int realCount = 0;
  for (size_t i = 0; i < 50;) {
    cv::Point3f pt3D;
    if (i == 0 ) {
       pt3D = cv::Point3f(0.,0.,1.);
    } else {
       pt3D = cv::Point3f(pt3Dx(gen),pt3Dy(gen),pt3Dz(gen));
    }
    cv::Point2f refCorner = cam->project(pt3D,rcw0,CtW0);
    cv::Point2f curCorner = cam->project(pt3D,rcw1,CtW1);
    if (!cam->isInFrame(refCorner) || !cam->isInFrame(curCorner)) {
      realCount++;
      if (realCount > 10000) {
        std::cerr << "Please input new pose of current frame,because current frame can't see points." << std::endl;
        return -1;
      }
      continue;
    }
    i++;
    refCorners[i] = refCorner;
    curCorners[i] = curCorner;
    std::cout << refCorner << " vs " << curCorner << std::endl;
  }
  std::map<uint64,cv::Point3f> pt3DMap;
  std::cout << "Real Pose:" << std::endl;
  std::cout << "Cur R :\n" << curFrame->Rwc() << std::endl;
  std::cout << "Cur t :\n" << curFrame->WtC() << std::endl;
  ini.initializeFromHomography(refFrame,curFrame,pt3DMap);
  std::cout << "Initial Pose:" << std::endl;
  std::cout << "Cur R :\n" << curFrame->Rwc() << std::endl;
  std::cout << "Cur t :\n" << curFrame->WtC() << std::endl;
  return 0;
}
