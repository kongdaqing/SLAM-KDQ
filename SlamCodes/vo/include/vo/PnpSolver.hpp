#pragma
#include <opencv2/calib3d.hpp>
namespace ov {
class PnpSolver {
 public:
  PnpSolver(){};

  /** \brief Solve pose using pnp solver ransac method
   * @param normalizedUV  ---  normalized features vector  
   * @param matchedPts3D  ---  matched features' 3D vector
   * @param focalLength   ---  camera focal length
   * @param R             ---  Rotation Matrix 
   * @param t             ---  translate vec 
   * @param inlier        ---  inlier vector of reproject 
   * @return return success otherwise false
   */ 
  bool solveByPnp(const std::vector<cv::Point2f>& normalizedUV,
                  const std::vector<cv::Point3f>& matchedPts3D,
                  double focalLength,cv::Mat &R,cv::Mat &t,
                  std::vector<uchar> &inlier) {
    if (normalizedUV.size() < 4) {
      return false;
    }
    cv::Mat K = cv::Mat::eye(3,3,CV_64F);
    return cv::solvePnPRansac(matchedPts3D,normalizedUV,K,cv::Mat(),R,t,false,50,8.0/focalLength,0.99,inlier);
  }

};
}