#pragma once
#include <opencv2/calib3d.hpp>
namespace ov {
class PnpSolver {
 public:
  PnpSolver(){};

  /** \brief Solve pose using pnp solver ransac method
   * @param normalizedUV  ---  normalized features vector  
   * @param matchedPts3D  ---  matched features' 3D vector
   * @param focalLength   ---  camera focal length
   * @param R             ---  Rotation Matrix from world to camera coordinate
   * @param t             ---  translate vec from world to camera coordinate
   * @param inlier        ---  inlier vector of reproject 
   * @param ratio         ---  rotio of inlier/total
   * @return return success otherwise false
   */ 
  bool solveByPnp(const std::vector<cv::Point2f>& normalizedUV,
                  const std::vector<cv::Point3f>& matchedPts3D,
                  double focalLength,cv::Mat &R,cv::Mat &t,
                  std::vector<int> &inlier,double ratio) {
    if (normalizedUV.size() < 4) {
      return false;
    }
    cv::Mat K = cv::Mat::eye(3,3,CV_64F);
    std::cout << focalLength << std::endl;
    bool resultFlg = cv::solvePnPRansac(matchedPts3D,normalizedUV,K,cv::Mat(),R,t,false,100,8.0/focalLength,0.9,inlier);
    
#ifdef VERBOOSE    
    std::cout << "result flg = " << resultFlg << ",Inlier" << inlier.size() << ":\n" ;
    for (int i = 0; i < matchedPts3D.size(); i++) {
      cv::Affine3d T(R,t);
      cv::Point3f ptInCam =  T * matchedPts3D[i];
      std::cout << ptInCam/ptInCam.z << " vs " << normalizedUV[i] << std::endl;
    }
#endif
    return resultFlg && inlier.size() > ratio * normalizedUV.size();
  }

};
}