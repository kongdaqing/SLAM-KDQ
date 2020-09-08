#include "inc/viz_scene/viz_scene.hpp"
namespace viz_scene {
  VizScene::VizScene() {

  }
  
  VizScene::~VizScene() {

  }
  
  bool VizScene::createRandomPlanePoints(string name,const Vec3f& center_vec,const Vec3f& normal_vec,int nums,int width,int height) {
    if(sceneCloud_.cloudMap.count(name)) {
      return false;
    }
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(1,6);
    Mat cloudPoints(1,nums,CV_32FC3);
    Point3f* point = cloudPoints.ptr<Point3f>(); 
    for(int i = 0;i < nums;i++) {
      point[i].x = dis(gen);
      point[i].y = dis(gen);
      point[i].z = 0;
    }
    sceneCloud_.cloudMap[name] = cloudPoints;
    return true;
  }


}



