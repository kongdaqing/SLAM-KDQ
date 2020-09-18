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
  
  void VizScene::showScenePointClouds() {
    for (auto it = sceneCloud_.begin(); it != sceneCloud_.end();it++) {
      if(it->second.updateFlg_ == false) {
        continue;
      }
      viz::WCloud widgetCloud(it->second.cloudPoints_,it->second.color_);
      sceneWindowPtr_->showWidget(it->first,widgetCloud);
    }
  }

  bool VizScene::createRandomPlanePoints(string name,const Vec3f& center_vec,const Vec3f& normal_vec,int nums,float width,float height,float thick) {
    if(sceneCloud_.count(name)) {
      return false;
    }
    assert(width > 0 && height > 0 && nums > 0);
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<float> width_dis(center_vec[0],width);//设置均值和标准差
    normal_distribution<float> height_dis(center_vec[1],height);
    normal_distribution<float> thick_dis;    
    if(thick > 0) {
      normal_distribution<float>::param_type para(center_vec[2],thick);
      thick_dis.param(para);
    }
    
    Mat cloudPoints(1,nums,CV_32FC3);
    Point3f* point = cloudPoints.ptr<Point3f>(); 
    for(int i = 0;i < nums;i++) {
      point[i].x = width_dis(gen);
      point[i].y = height_dis(gen);
      if(point[i].x > width || point[i].x < -width || point[i].y > height || point[i].y < -height) {
        i--;
        continue;
      }
      if(thick <= 0) {
        if(normal_vec[2] != 0) {
          point[i].z = -(normal_vec[0]/normal_vec[2])*(point[i].x - center_vec[0]) -
                      (normal_vec[1]/normal_vec[2])*(point[i].y - center_vec[1]) + center_vec[2]; 
        } else {
          point[i].z = center_vec[2];
        }
      } else {
        point[i].z = thick_dis(gen);
      }
    }
    ScenePointCloud sceneCloudObject(cloudPoints,viz::Color::green());
    sceneCloud_[name] = sceneCloudObject;
    viz::WCloud widgetCloud(sceneCloud_[name].cloudPoints_,sceneCloud_[name].color_);
    sceneWindowPtr_->showWidget(name,widgetCloud);
    return true;
  }

  bool VizScene::createCameraObject(string cameraName,float coorScalar,Vec2f frustumScalar,Vec3f pos,Vec3f focalPointPos,Vec3f y_direction) {
    if(sceneCamera_.count(cameraName)) {
      return false;
    }
    CameraObject camera(cameraName,coorScalar,frustumScalar,pos,focalPointPos,y_direction,30);
    sceneCamera_[cameraName] = camera;
    sceneWindowPtr_->showWidget(sceneCamera_[cameraName].cCoorScalarName_,
                    sceneCamera_[cameraName].cameraCoordinateScalar_,
                    sceneCamera_[cameraName].cameraPose_);
    sceneWindowPtr_->showWidget(sceneCamera_[cameraName].cFrusName_,
                    sceneCamera_[cameraName].cameraFrustum_,
                    sceneCamera_[cameraName].cameraPose_);
    return true;
  }

  bool VizScene::updateCameraPose(const string cameraName,const Affine3d& cameraPose) {
    lock_guard<mutex> lockWCloud(mCloud_);
    if(!sceneCamera_.count(cameraName)) {
      return false;
    }
    sceneCamera_[cameraName].updatePose(cameraPose);
    viz::WTrajectory cameraPath(sceneCamera_[cameraName].poseVec_);
    sceneWindowPtr_->showWidget(sceneCamera_[cameraName].cPathName_,cameraPath);
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

  cv::Mat VizScene::getScenePointCloud(string name) {
    cv::Mat ans;
    if (sceneCloud_.count(name)) {
      ans = sceneCloud_[name].cloudPoints_.clone();
    }
    return ans;
  }
}



