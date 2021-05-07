#include "VizScene.hpp"

namespace ov {
VizScene::VizScene(std::string windowName) {
  sceneCloud_.clear();
  sceneWindowPtr_ = new cv::viz::Viz3d(windowName);
  sceneWindowPtr_->showWidget("widget coordinate",cv::viz::WCoordinateSystem());
#ifndef __APPLE__
  //windowLoopThread_ = new std::thread(&VizScene::windowShowLoopRun,this);
#endif
  //windowLoopThread_->join();//如果主线程执行很快就结束，必须加join来强行插入进程，否则主线程很快直接运行结束，导致无法运行该线程
}

VizScene::~VizScene() {
  sceneCloud_.clear();
  sceneCamera_.clear();
  delete windowLoopThread_;
  delete sceneWindowPtr_;
}
void VizScene::windowShowLoopRun() {
  while (!sceneWindowPtr_->wasStopped()) {
    mCloud_.lock();
    sceneWindowPtr_->spinOnce(1,false);
    mCloud_.unlock();
    usleep(10);
  }
}

void VizScene::showSceneAllPointClouds() {
  for (auto it = sceneCloud_.begin(); it != sceneCloud_.end();it++) {
    if(it->second.updateFlg_ == false) {
      continue;
    }
    cv::viz::WCloud widgetCloud(it->second.cloudPoints_,it->second.color_);
    widgetCloud.setRenderingProperty(cv::viz::POINT_SIZE,it->second.displaySize_);
    sceneWindowPtr_->showWidget(it->first,widgetCloud);
    it->second.updateFlg_ = false;
  }
}

void VizScene::showSceneAllCamera() {
  for (auto it = sceneCamera_.begin(); it != sceneCamera_.end(); it++) {
    if (it->second.poseUpdateFlg_ == false) {
      continue;
    }
    sceneWindowPtr_->showWidget(it->second.cCoorScalarName_,
                                it->second.cameraCoordinateFrame_,
                                it->second.cameraPose_);
    sceneWindowPtr_->showWidget(it->second.cFrusName_,
                                it->second.cameraFrustum_,
                                it->second.cameraPose_);
    cv::viz::WTrajectory cameraPath(it->second.path_);
    sceneWindowPtr_->showWidget(it->second.cPathName_,cameraPath);
    it->second.poseUpdateFlg_ = false;
  }
}

bool VizScene::createPointClouds(std::string ptsName,std::vector<cv::Vec3f>& pts3D,cv::viz::Color color,int displaySize) {
  if (pts3D.empty()) {
    printf("Input points vector is empty!\n");
    return false;
  }
  if (sceneCloud_.count(ptsName)) {
    printf("%s pointcloud is already created!\n",ptsName.c_str());
    return false;
  }
  ScenePointCloud pc(pts3D,color,displaySize);
  sceneCloud_[ptsName] = pc;  
  return true;
}

bool VizScene::createCameraObject(std::string cameraName,
                                  float coorScalar,
                                  float frustumScalar,
                                  cv::Vec2f fov,
                                  cv::Vec3f pos,
                                  cv::Vec3f focalPointPos,
                                  cv::Vec3f yDirection) {
  if(sceneCamera_.count(cameraName)) {
    printf("camera %s is already create!\n",cameraName.c_str());
    return false;
  }
  CameraObject camera(cameraName,coorScalar,frustumScalar,fov,pos,focalPointPos,yDirection,3);
  sceneCamera_[cameraName] = camera;
  return true;
}

bool VizScene::updatePointClouds(std::string ptsName,const std::vector<cv::Vec3f>& pts) {
  std::lock_guard<std::mutex> lockWCloud(mCloud_);
  if (!sceneCloud_.count(ptsName)) {
    printf("No %s pointcloud exist!\n",ptsName.c_str());
    return false;
  }
  sceneCloud_[ptsName].update(pts);
  return true;
}

bool VizScene::updateCameraPose(std::string cameraName,const cv::Affine3d& cameraPose) {
  std::lock_guard<std::mutex> lockWCloud(mCloud_);
  if(!sceneCamera_.count(cameraName)) {
    printf("Camera %s is not exist!\n",cameraName.c_str());
    return false;
  }
  sceneCamera_[cameraName].updatePose(cameraPose);
  return true;
}
}



