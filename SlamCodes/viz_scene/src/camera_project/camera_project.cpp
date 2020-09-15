#include <map>
#include "camera_project/camera_project.hpp"
#include "utilities/access_file.hpp"
CameraProject::CameraProject(std::string cameraConfigFile,std::string outputPtsFile) {
  camPtr_ = CameraFactory::instance()->generateCameraFromYamlFile(cameraConfigFile);
  outPtsFile = outputPtsFile;
}

void CameraProject::projectVizPoints(double t,const cv::Mat& ptsCloud,const Eigen::Isometry3d& cameraPos) {
  if(ptsCloud.empty()) {
    return;
  }
  cv::Mat ptsClouds(1,ptsCloud.cols,CV_32FC3);
  ptsClouds = ptsCloud.clone();
  cv::Point3f* point = ptsClouds.ptr<cv::Point3f>();
  std::map<int,Eigen::Matrix<double,5,1> > ptsMap;
  for (size_t i = 0; i < ptsClouds.cols; i++) {
    Eigen::Vector3d pW(point[i].x,point[i].y,point[i].z);
    Eigen::Vector3d pC = cameraPos * pW;
    Eigen::Vector2d pixel;
    camPtr_->spaceToPlane(pC,pixel);
    if (inBorder(pixel)) {
      Eigen::Matrix<double,5,1> ptsInfo;
      ptsInfo << pW.x() , pW.y() , pW.z() , pixel.x() , pixel.y();
      ptsMap[i] = ptsInfo;
    }
  }
  VioDatasInterface::recordCameraPixel(t,ptsMap);
}

bool CameraProject::inBorder(const Eigen::Vector2d& pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x());
    int img_y = cvRound(pt.y());
    return BORDER_SIZE <= img_x && img_x < camPtr_->imageWidth - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < camPtr_->imageHeight - BORDER_SIZE;
}
