#pragma
#include "Config.hpp"
#include "FeatureManager.hpp"

namespace ov {

class Initializator {
 public:
  Initializator(const Config* cfg,Camera* cam);

  bool initPoseAndMap(Frame* refFrame,Frame* curFrame,FeatureManager& fm);

  /** \brief Calculate pose of two frames with overlap features through decomposing homography matrix 
   *  @param T           --- output transform matrix 
   *  @param refFeatures --- pixel points in referance frame
   *  @param curFeatures --- pixel points in curent frame
   *  @return  success flag of calculating pose  
   */ 
  bool initializeFromHomography(Frame* refFrame,Frame* curFrame,std::map<uint64,cv::Point3f>& pts3D);

  /** \brief Calculate homography matrix of two frames with overlap features  
   *  @param H           --- homegraphy matrix
   *  @param inliers     --- inlier flg array that reprojected error less than @ReprojectErrThr_
   *  @param K           --- camera intrinisic martrix 
   *  @param refFeatures --- pixel points in referance frame
   *  @param curFeatures --- pixel points in curent frame
   *  @return  success flag of calculating homography
   */ 
  bool calHomography(cv::Mat &H,
                     std::vector<uchar> &inliers,
                     const cv::Mat &K,
                     const std::vector<cv::Point2f> &refFeatures,
                     const std::vector<cv::Point2f> &curFeatures);

  /** \brief select best pose from pose vector through check all features 3d point is or not in the front of cameras
   *  @param R         --- Rotation Matrix Array from reference to current frame
   *  @param t         --- translate Matrix Array from reference to current frame
   *  @param n         --- normal Vector Array
   *  @param K         --- camera intrinsic matrix
   *  @param refFeatures --- corner point array in reference frame
   *  @param curFeatures --- corner point array matched with reference in the current frame
   */ 
  int checkRt(std::vector< std::map<uint64,cv::Point3f> > &ptsInWorld,
                        const std::vector<cv::Mat> &R,
                        const std::vector<cv::Mat> &t,
                        const std::vector<cv::Mat> &n,
                        const cv::Mat &K,
                        const std::vector<uchar> &inliers,
                        const std::vector<uint64> &idVec,
                        const std::vector<cv::Point2f> &refFeatures,
                        const std::vector<cv::Point2f> &curFeatures);
  /** \brief Check homography properity 
   *  @param H --- homography matrix
   *  @return homography is ok or not
   */ 
  inline bool checkHomography(const cv::Mat &H); 

  /** \brief get 3D coordinate through triangulating feature points 
   * @param kp1  --- pixel coordinate in the first frame 
   * @param kp2  --- pixel coordinate in the second frame
   * @param P1   --- pose of first frame in the world 
   * @param P2   --- pose of second frame in the world
   * @param x3D  --- point coordinate in the world
   */ 
  void triangulate(const cv::Point2f &kp1,    
                             const cv::Point2f &kp2,    
                             const cv::Mat &P1,         
                             const cv::Mat &P2,          
                             cv::Point3f &pts3d);

  /** \brief check matched corners middle disparitie,make sure this value not too small
   * @param refCorners  --- corner vector in the reference frame
   * @param curCorners  --- corner vector in the current frame 
   */ 
  bool checkCornerDisparities(std::vector<cv::Point2f>& refCorners,
                         std::vector<cv::Point2f>& curCorners);
 private:
  Camera *camera_;
  const float MinDisparity;
  const int InitialMinMatchedPointNum;
  const float InitialReprojectErr;
  const float HomographyTransformErr;
};
}

