#pragma once
#include <opencv2/core.hpp>
#include "Config.hpp"
#include "Frame.hpp"
#include "Camera.hpp"
namespace ov {
class FeatureTracker{
 public:

  /** \brief construct
   */ 
  FeatureTracker(const Config* cfg);

  /** \brief detect new features and track last features in current frame
   * @param refFrame  --- reference frame ptr
   * @param curFrame  --- current frame ptr
   */ 
  void detectAndTrackFeature(FramePtr refFrame,FramePtr curFrame);
  
 private:
  const int MaxPointSize;
  const float QualityLevel;
  const float MinDist;
  const int TrackBack;
  const int PyramidLevel;
  const int CriterIterations;
  const double CriterEPS;
  Camera* cam_; 

  uint64 id_;

  /** \brief remove bad data in the vector
   * @param status  ---  status vector 1 is good,0 needs remove
   * @param data    ---  vector data 
   */ 
  template <class T>
  inline void remove(const std::vector<uchar>& status,std::vector<T>& data) {
    assert(status.size() == data.size());
    size_t i = 0,j = 0;
    for (i = 0; i < status.size(); i++) {
      if (status[i]) {
        data[j] = data[i];
        j++;
      }
    } 
    data.resize(j);
  }

  /** \brief set mask at given features
   * @param img  ---  image size
   * @param feat ---  vector of featurs
   */ 
  cv::Mat setMask(const cv::Mat& img,const std::vector<cv::Point2f>& feat) {
    cv::Mat mask(cv::Size(img.cols,img.rows),CV_8UC1,255);
    if (feat.empty()) {
      return mask;
    }
    for (size_t i = 0; i < feat.size(); i++) {
      if (mask.at<uchar>(feat[i]) == 255) {
        cv::circle(mask,feat[i],MinDist,0,-1);
      }
    }
    return mask;
  } 
};
}


