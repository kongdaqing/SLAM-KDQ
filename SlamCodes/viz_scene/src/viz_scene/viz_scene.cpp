#include "viz_scene/viz_scene.hpp"
#include "timer/timer.hpp"

namespace viz_scene {
  VizScene::VizScene(string windowName) {
    sceneCloud_.clear();
    sceneWindowPtr_ = new viz::Viz3d(windowName);
    sceneWindowPtr_->showWidget("widget coordinate",viz::WCoordinateSystem());
    windowLoopThread_ = new thread(&VizScene::windowShowLoopRun,this);
    //windowLoopThread_->join();//如果主线程执行很快就结束，必须加join来强行插入进程，否则主线程很快直接运行结束，导致无法运行该线程
  }
  
  VizScene::~VizScene() {
    sceneCloud_.clear();
    delete sceneWindowPtr_;
  }

  void VizScene::windowShowLoopRun() {
    while (!sceneWindowPtr_->wasStopped()) {
      mCloud_.lock();
      sceneWindowPtr_->spinOnce(1,false);
      mCloud_.unlock();
      usleep(100);
    }
  }
  
  void VizScene::showScenePointClouds() {
    for (auto it = sceneCloud_.begin(); it != sceneCloud_.end();it++) {
      if(it->second.updateFlg_ == false) {
        continue;
      }
      viz::WCloud widgetCloud(it->second.cloudPoints_,it->second.color_);
      sceneWindowPtr_->showWidget(it->first,widgetCloud);
    }
  }

  bool VizScene::createRandomPlanePoints(string name,const Vec3f& center_vec,const Vec3f& normal_vec,int nums,int width,int height) {
    if(sceneCloud_.count(name)) {
      return false;
    }
    assert(width > 0 && height > 0 && nums > 0);
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<float> width_dis(center_vec[0],width);//设置均值和标准差
    normal_distribution<float> height_dis(center_vec[1],height);
    Mat cloudPoints(1,nums,CV_32FC3);
    Point3f* point = cloudPoints.ptr<Point3f>(); 
    for(int i = 0;i < nums;i++) {
      point[i].x = width_dis(gen);
      point[i].y = height_dis(gen);
      if(normal_vec[2] != 0) {
        point[i].z = -(normal_vec[0]/normal_vec[2])*(point[i].x - center_vec[0]) -
                    (normal_vec[1]/normal_vec[2])*(point[i].y - center_vec[1]) + center_vec[2]; 
      } else {
        point[i].z = center_vec[2];
      }
    }
    ScenePointCloud sceneCloudObject(cloudPoints,viz::Color::green());
    sceneCloud_[name] = sceneCloudObject;
    viz::WCloud widgetCloud(sceneCloud_[name].cloudPoints_,sceneCloud_[name].color_);
    sceneWindowPtr_->showWidget(name,widgetCloud);
    return true;
  }

  void VizScene::testIncreasePoints(string name) {
    lock_guard<mutex> lockWCloud(mCloud_);
    if(!sceneCloud_.count(name)) {
      cout << "no this " << name << endl;
      return;
    }
    Timer ticToc;
    ScenePointCloud& ptsCloud = sceneCloud_[name];
    for (size_t i = 0; i < ptsCloud.cloudPoints_.cols; i++)
    {
      Point3f* pt = ptsCloud.cloudPoints_.col(i).ptr<Point3f>();
      pt->z += 0.01;
    }
    viz::WCloud widgetCloud(sceneCloud_[name].cloudPoints_,sceneCloud_[name].color_);
    sceneWindowPtr_->showWidget(name,widgetCloud);
  }
}



