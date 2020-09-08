#include <iostream>
#include <map>
#include <random>
#include <opencv2/opencv.hpp>
namespace viz_scene {
using namespace std;
using namespace cv;

  struct ScenePointCloud{
    map<string,Mat> cloudMap;
  };

  class VizScene {
    private:
      ScenePointCloud sceneCloud_;
    public:
      VizScene();
      ~VizScene();
      /** \brief create plane filled with random points  
       * @param name - name of scene pointcloud set by user  
       * @param center_vec - center position of this plane scene in world coordinate
       * @param normal_vec - normal vector of this plane
       * @param nums - number of point cloud
       * @param width - width of plane
       * @param height - height of plane
       */
      bool createRandomPlanePoints(string name,const Vec3f& center_vec,const Vec3f& normal_vec,int nums,int width,int height);
  };
}