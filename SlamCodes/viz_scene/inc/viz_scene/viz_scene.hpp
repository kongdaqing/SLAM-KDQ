#include <iostream>
#include <map>
#include <random>
#include <thread>
#include <mutex>
#include <unistd.h>
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

  class VizScene {
    private:
      map<string,ScenePointCloud> sceneCloud_;
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
       */
      bool createRandomPlanePoints(string name,const Vec3f& center_vec,const Vec3f& normal_vec,int nums,int width,int height);

      /** \brief test viz window display cloud points during increase position
       * 
       */
      void testIncreasePoints(string name);
  };
}