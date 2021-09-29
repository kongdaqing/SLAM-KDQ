#include "viz_scene/viz_scene.hpp"
#include "timer/timer.hpp"

namespace viz_scene {
VizScene::VizScene(string windowName, double scale) {
  sceneCloud_.clear();
  sceneWindowPtr_ = new viz::Viz3d(windowName);
  sceneWindowPtr_->showWidget("widget coordinate", viz::WCoordinateSystem(scale));
  windowLoopThread_ = new thread(&VizScene::windowShowLoopRun, this);
  //windowLoopThread_->join();//如果主线程执行很快就结束，必须加join来强行插入进程，否则主线程很快直接运行结束，导致无法运行该线程
}

VizScene::~VizScene() {
  sceneCloud_.clear();
  sceneCamera_.clear();
  sceneWindowPtr_->close();
  if (windowLoopThread_->joinable()) {
    windowLoopThread_->join();
  }
  delete sceneWindowPtr_;
  delete windowLoopThread_;
}

void VizScene::windowShowLoopRun() {
  while (!sceneWindowPtr_->wasStopped()) {
    mCloud_.lock();
    sceneWindowPtr_->spinOnce(1, false);
    mCloud_.unlock();
    usleep(10);
  }
}

void VizScene::showScenePointClouds() {
  for (auto it = sceneCloud_.begin(); it != sceneCloud_.end(); it++) {
    if (it->second.updateFlg_ == false) {
      continue;
    }
    viz::WCloud widgetCloud(it->second.cloudPoints_, it->second.color_);
    sceneWindowPtr_->showWidget(it->first, widgetCloud);
  }
}

bool VizScene::createRandomPlanePoints(string name,
                                       const Vec3f &center_vec,
                                       const Vec3f &normal_vec,
                                       int nums,
                                       float width,
                                       float height,
                                       float thick) {
  if (sceneCloud_.count(name)) {
    return false;
  }
  assert(width > 0 && height > 0 && nums > 0);
  random_device rd;
  mt19937 gen(rd());
  normal_distribution<float> width_dis(center_vec[0], width);//设置均值和标准差
  normal_distribution<float> height_dis(center_vec[1], height);
  normal_distribution<float> thick_dis;
  if (thick > 0) {
    normal_distribution<float>::param_type para(center_vec[2], thick);
    thick_dis.param(para);
  }
  vector<Vec3f> pointCloud;
  for (int i = 0; i < nums; i++) {
    Vec3f point;
    point[0] = width_dis(gen);
    point[1] = height_dis(gen);
    if (point[0] > width || point[0] < -width || point[1] > height || point[1] < -height) {
      i--;
      continue;
    }
    if (thick <= 0) {
      if (normal_vec[2] != 0) {
        point[2] = -(normal_vec[0] / normal_vec[2]) * (point[0] - center_vec[0]) -
            (normal_vec[1] / normal_vec[2]) * (point[1] - center_vec[1]) + center_vec[2];
      } else {
        point[2] = center_vec[2];
      }
    } else {
      point[2] = thick_dis(gen);
    }
    pointCloud.push_back(point);
  }
  ScenePointCloudType sceneCloudObject(pointCloud, viz::Color::green());
  sceneCloud_[name] = sceneCloudObject;
  viz::WCloud widgetCloud(sceneCloud_[name].cloudPoints_, sceneCloud_[name].color_);
  sceneWindowPtr_->showWidget(name, widgetCloud);
  return true;
}
bool VizScene::createDynamicClouds(std::string ptsName,
                                   std::vector<cv::Vec3f> &pts3D,
                                   cv::viz::Color color,
                                   int displaySize) {
  if (pts3D.empty()) {
    printf("Input points vector is empty!\n");
    return false;
  }
  if (sceneCloud_.count(ptsName)) {
    printf("%s pointcloud is already created!\n", ptsName.c_str());
    return false;
  }
  ScenePointCloudType pc(pts3D, color, displaySize);
  sceneCloud_[ptsName] = pc;
  return true;
}

bool VizScene::updateDynamicClouds(std::string ptsName, const std::vector<cv::Vec3f> &pts3D) {
  std::lock_guard<std::mutex> lockWCloud(mCloud_);
  if (!sceneCloud_.count(ptsName)) {
    printf("No %s pointcloud exist!\n", ptsName.c_str());
    return false;
  }
  if (pts3D.empty()) {
    printf("PointCloud empty!\n");
    return false;
  }
  sceneCloud_[ptsName].update(pts3D);
  return true;
}

bool VizScene::createCameraObject(string cameraName,
                                  float coorScalar,
                                  Vec2f frustumScalar,
                                  Vec3f pos,
                                  Vec3f focalPointPos,
                                  Vec3f y_direction) {
  if (sceneCamera_.count(cameraName)) {
    return false;
  }
  CameraObject camera(cameraName, coorScalar, frustumScalar, pos, focalPointPos, y_direction, 30);
  sceneCamera_[cameraName] = camera;
  sceneWindowPtr_->showWidget(sceneCamera_[cameraName].cCoorScalarName_,
                              sceneCamera_[cameraName].cameraCoordinateScalar_,
                              sceneCamera_[cameraName].cameraPose_);
  sceneWindowPtr_->showWidget(sceneCamera_[cameraName].cFrusName_,
                              sceneCamera_[cameraName].cameraFrustum_,
                              sceneCamera_[cameraName].cameraPose_);
  return true;
}

bool VizScene::updateCameraPose(const string cameraName, const Affine3d &cameraPose) {
  lock_guard<mutex> lockWCloud(mCloud_);
  if (!sceneCamera_.count(cameraName)) {
    return false;
  }
  sceneCamera_[cameraName].updatePose(cameraPose);
  viz::WTrajectory cameraPath(sceneCamera_[cameraName].poseVec_);
  sceneWindowPtr_->showWidget(sceneCamera_[cameraName].cPathName_, cameraPath);
}

void VizScene::testIncreasePoints(string name) {
  lock_guard<mutex> lockWCloud(mCloud_);
  if (!sceneCloud_.count(name)) {
    cout << "no this " << name << endl;
    return;
  }
  Timer ticToc;
  ScenePointCloudType &ptsCloud = sceneCloud_[name];
  for (size_t i = 0; i < ptsCloud.cloudPoints_.size(); i++) {
    Vec3f *pt = &ptsCloud.cloudPoints_[i];
    (*pt)[2] += 0.01;
  }
  viz::WCloud widgetCloud(sceneCloud_[name].cloudPoints_, sceneCloud_[name].color_);
  sceneWindowPtr_->showWidget(name, widgetCloud);
}

vector<Vec3f> VizScene::getScenePointCloud(string name) {
  vector<Vec3f> cloud;
  if (sceneCloud_.count(name)) {
    cloud = sceneCloud_[name].cloudPoints_;
  }
  return cloud;
}
}



