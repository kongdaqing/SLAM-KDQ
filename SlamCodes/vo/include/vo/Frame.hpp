#pragma once 
#include <memory>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>

namespace ov {



class Frame {
 public:

  /** \brief construct 
   */ 
  Frame() {};
  /** \brief construct function
   * @param timestamp --- timestamp of current frame
   * @param img       --- image data 
   */ 
  Frame(double timestamp,cv::Mat& img): timestamp_(timestamp) {
    image_ = img.clone();
    cv::Mat R = cv::Mat::eye(3,3,CV_64F);
    cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
    setPoseInWorld(R,t);
  };

  /** \brief construct function
   * @param timestamp --- timestamp of current frame 
   * @param corners   --- corners map 
   */ 
  Frame(double timestamp,std::map<uint64,cv::Point2f>& corners): 
  timestamp_(timestamp),corners_(corners) {
    cv::Mat R = cv::Mat::eye(3,3,CV_64F);
    cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
    setPoseInWorld(R,t);
  }
  
  /** \brief initializing with frame object
   */ 
  Frame(const Frame& frame) {
    timestamp_ = frame.timestamp_;
    corners_ = frame.getCorners();
    image_ = frame.image_.clone();
    frame.getPoseInWorld(Rwc_,WtC_);
  }

  /** \brief unconstruct function 
   * 
   */ 
  ~Frame() {
    //corners_.clear();//自动调用析构函数
    //image_.release();//CV::Mat会自动调用析构函数
  }

  /** \brief set frame pose in the world frame
   * @param Rwc  ---  rotation matrix from frame to world
   * @param WtC  ---  translate matrix from frame to world 
   */ 
  void setPoseInWorld(cv::Mat Rwc,cv::Mat WtC) {
    Rwc.copyTo(Rwc_);
    WtC.copyTo(WtC_);
  }

  /** \brief get pose from frame to world
   * @param Rwc   --- rotation matrix from frame to world 
   * @param WtC   --- translate vector from frame to world
   */ 
  void getPoseInWorld(cv::Mat& Rwc,cv::Mat& WtC) const{
    Rwc_.copyTo(Rwc);
    WtC_.copyTo(WtC);
  }

  /** \brief get pose from world to frame
   * @param Rcw   --- rotation matrix from world to frame 
   * @param CtW   --- translate vector from world to frame
   */ 
  void getInversePose(cv::Mat& Rcw,cv::Mat& CtW) const{
    Rcw = Rwc_.t();
    CtW = -Rcw * WtC_;
  }
  
  /** \brief  set corners map
   * @param corners  --- corners map
   */ 
  void setCorners(std::map<uint64,cv::Point2f>& corners) {
    corners_ = corners;
  };

  /** \brief get corners map
   * @return corners map in the this frame
   */ 
  const std::map<uint64,cv::Point2f> &getCorners() const{
    return corners_;
  }

  /** \brief get corners map
   * @return corners map in the this frame
   */ 
  std::map<uint64,cv::Point2f> &getCorners() {
    return corners_;
  }

  std::map<uint64,cv::Point2f> getCornersCopy() {
    return corners_;
  }

  /** \brief get corners vector 
   * @param idx     ---  vector of feature id
   * @param corners ---  vector of feature uv corresponding id
   */ 
  void getCornerVector(std::vector<uint64> &idx,std::vector<cv::Point2f> &corners) {
    for (auto it = corners_.begin(); it != corners_.end(); it++) {
      idx.push_back(it->first);
      corners.push_back(it->second);
    }
  }

  /** \brief set corners map 
   * @param idx     ---  vector of feature id
   * @param corners ---  vector of feature uv corresponding id
   */ 
  void setCornerMap(const std::vector<uint64> &idx,const std::vector<cv::Point2f> &corners) {
    for (size_t i = 0; i < idx.size(); i++) {
      corners_[idx[i]] = corners[i];
    }
  }

  /** \brief get corner pixel coordinate corresponding id 
   * @param id --- corner id
   * @param uv --- pixel coordinate reference 
   * @return return false if no this id 
   */   
  bool getCornerUV(uint64_t id,cv::Point2f& uv) const{
    if (!corners_.count(id)) {
      return false;
    }
    uv = corners_.at(id);
    return true;
  }

  /** \brief get all both matched features with reference frame
   * @param  refFrame  --- reference frame 
   * @param matchedId  --- matched id vector
   * @param refCorners --- matched pixel vector in the reference frame
   * @param curCorners --- matched pixel vector in the current frame
   */ 
  void getMatchedFeatures(const Frame* refFrame,
                          std::vector<uint64> &matchedId,
                          std::vector<cv::Point2f> &refCorners,
                          std::vector<cv::Point2f> &curCorners) {
   const std::map<uint64,cv::Point2f> refFeats = refFrame->getCorners();
    for (auto it = refFeats.begin();it != refFeats.end();it++) {
      if (corners_.count(it->first)) {
        matchedId.push_back(it->first);
        refCorners.push_back(it->second);
        curCorners.push_back(corners_[it->first]);
      }
    }
  }

  void imshowFeatures(uint scale) {
    if (image_.empty()) {
      return;
    }
    cv::Mat colorImage;
    cv::cvtColor(image_,colorImage,cv::COLOR_GRAY2BGR);
    for (auto it = corners_.begin(); it != corners_.end(); it++) {
      cv::Point2f uv = it->second;
      cv::circle(colorImage,uv,2,cv::Scalar(0,255,0));
      cv::putText(colorImage,std::to_string(it->first),uv,cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,0));
    }
    cv::resize(colorImage,colorImage,cv::Size(colorImage.cols*scale,colorImage.rows*scale));
    cv::imshow("test_track",colorImage);
    cv::waitKey(1);
  }

  /** \brief get rotation matrix
   */ 
  inline cv::Mat Rwc() const {
    return Rwc_;
  }

  /** \brief get translate part
   */ 
  inline cv::Mat WtC() const {
    return WtC_;
  }
  cv::Mat image_;
  double timestamp_;
 private:
  cv::Mat Rwc_,WtC_; // rotation matrix and translate vector from camera to world
  std::map<uint64,cv::Point2f> corners_;
};

typedef std::shared_ptr<Frame> FramePtr;

}