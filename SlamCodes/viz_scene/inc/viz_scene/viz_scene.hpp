#pragma once
#include <iostream>
#include <map>
#include <random>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <vector>
#include <opencv2/opencv.hpp>
namespace viz_scene {
using namespace std;
using namespace cv;

  
  struct ScenePointCloud{
    Mat cloudPoints_;
    bool updateFlg_;
    viz::Color color_;
    
    ScenePointCloud() {
      updateFlg_ = false;
      color_ = viz::Color::white();
    }

    ScenePointCloud(const cv::Mat& cloudPts) {
      cloudPoints_ = cloudPts.clone();
      updateFlg_ = true;
      color_ = viz::Color::white();
    }

    ScenePointCloud(const cv::Mat& cloudPts,viz::Color color) {
      cloudPoints_ = cloudPts.clone();
      updateFlg_ = true;
      color_ = color;
    }

  };

  class CameraObject {
    public:
      CameraObject(){
      };
      CameraObject(string cameraName,float coorScalar,Vec2f frustumScalar,Vec3f pos,Vec3f focalPointPos,Vec3f y_dirction,float recordPathFreq):
      cameraCoordinateScalar_(coorScalar),cameraFrustum_(frustumScalar,1.0,viz::Color::green()){
        cameraPose_ = viz::makeCameraPose(pos,focalPointPos,y_dirction);
        cCoorScalarName_ = cameraName + "_coor";
        cFrusName_ = cameraName + "_frustum";
        cPathName_ = cameraName + "_path";
        poseVec_.clear();
        recoreFreq_ = recordPathFreq;
        recordCount_ = 0;
      };
      viz::WCameraPosition cameraCoordinateScalar_;
      viz::WCameraPosition cameraFrustum_;
      vector<Affine3d> poseVec_;
      Affine3d cameraPose_;
      string cCoorScalarName_,cFrusName_,cPathName_;
      void updatePose(const Affine3d& pos){
        cameraPose_ = pos;
        cameraCoordinateScalar_.setPose(cameraPose_);
        cameraFrustum_.setPose(cameraPose_);
        recordCount_++;
        if(recordCount_ % recoreFreq_ == 0)
          poseVec_.push_back(pos);
      };
      void clearPath() {
        poseVec_.clear();
        recordCount_ = 0;
      }
      private:
        int recoreFreq_;
        int recordCount_;
  };

  class VizScene {
    private:
      map<string,ScenePointCloud> sceneCloud_;
      map<string,CameraObject> sceneCamera_;
      viz::Viz3d* sceneWindowPtr_;
      thread* windowLoopThread_;
      mutex mCloud_;
    public:
      VizScene(string windowName);
      ~VizScene();

      /** \brief viz window show at spinOnce way
       * 
       */
      void windowShowLoopRun();

      /** \brief show pointclouds in parameter sceneCloud_
       */
      void showScenePointClouds();

      /** \brief create plane filled with random points  
       * @param name - name of scene pointcloud set by user  
       * @param center_vec - center position of this plane scene in world coordinate
       * @param normal_vec - normal vector of this plane
       * @param nums - number of point cloud
       * @param width - width of plane
       * @param height - height of plane
       * @param thick - if thick !=0,then create cube filled with random points
       */
      bool createRandomPlanePoints(string name,const Vec3f& center_vec,const Vec3f& normal_vec,int nums,float width,float height,float thick = 0);
    
      /** \brief create camera object  
       * @param cameraName - camera name   
       * @param coorScalar - scalar of camera coordinate
       * @param frustumScalar - frustum Scalar 
       * @param pos - camera position 
       * @param focalPointPos - focal point position
       * @param y_direction -  y axis direction of camera frame
       */
      bool createCameraObject(string cameraName,float coorScalar,Vec2f frustumScalar,Vec3f pos,Vec3f focalPointPos,Vec3f y_direction);

      /** \brief update pose of camera named by "cameraName"
       * @param cameraName - camera name
       * @param cameraPose - camera pose 
       */
      bool updateCameraPose(const string cameraName,const Affine3d& cameraPose);
      /** \brief test viz window display cloud points during increase position
       * 
       */
      /** \brief test viz window display cloud points during increase position
       * 
       */
      void testIncreasePoints(string name);


  };
}