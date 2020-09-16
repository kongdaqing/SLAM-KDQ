#include <map>
#include "camera_project/camera_project.hpp"
#include "utilities/access_file.hpp"
CameraProject::CameraProject(bool addNoise,float pixelNoise,std::string cameraConfigFile,std::string outputPtsFile) {
  camPtr_ = CameraFactory::instance()->generateCameraFromYamlFile(cameraConfigFile);
  outPtsFile_ = outputPtsFile;
  pixelNoise_ = pixelNoise;
  addNoise_ = addNoise;
}

void CameraProject::projectVizPoints(double t,const cv::Mat& ptsCloud,const Eigen::Isometry3d& cameraPos) {
  if(ptsCloud.empty()) {
    return;
  }
  std::mt19937 gen{12345};
  std::normal_distribution<> d{0.0, pixelNoise_};

  cv::Mat ptsClouds(1,ptsCloud.cols,CV_32FC3);
  ptsClouds = ptsCloud.clone();
  cv::Point3f* point = ptsClouds.ptr<cv::Point3f>();
  std::map<int,std::pair<cv::Point3d,cv::Point2i>> ptsMap;
  for (size_t i = 0; i < ptsClouds.cols; i++) {
    Eigen::Vector3d pW(point[i].x,point[i].y,point[i].z);
    Eigen::Vector3d pC = cameraPos * pW;
    Eigen::Vector2d pixel;
    if (pC.z() < 0) {
      continue;
    }
    camPtr_->spaceToPlane(pC,pixel);
    cv::Point2i uv;
    
    uv.x = pixel.x() + 0.5 + addNoise_? std::round(d(gen)) : 0.0;
    uv.y = pixel.y() + 0.5 + addNoise_? std::round(d(gen)) : 0.0;
    
    if (inBorder(pixel)) {
      ptsMap[i] = std::make_pair(cv::Point3d(pW.x(),pW.y(),pW.z()),uv);
    }

  }
  VioDatasInterface::recordCameraPixel(t,ptsMap,outPtsFile_);
}

bool CameraProject::inBorder(const Eigen::Vector2d& pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x());
    int img_y = cvRound(pt.y());
    return BORDER_SIZE <= img_x && img_x < camPtr_->imageWidth() - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < camPtr_->imageHeight() - BORDER_SIZE;
}
