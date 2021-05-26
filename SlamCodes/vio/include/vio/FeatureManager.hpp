#pragma once 
#include "Frame.hpp" 
#include "Camera.hpp"
#define WINSIZE 10

namespace vio{
class Feature {
 public:
  /** \brief construct function
   */
  Feature() {
    valid3D_ = false;
    badCount_ = 0;
  }

  /** \brief construct function include set id and add feature pixel
   * @param idx  --- id of feature
   * @param f    --- frame ptr
   */ 
  Feature(uint64 idx,Frame *f) {
    id = idx;
    addFrame(f);
    valid3D_ = false;
    badCount_ = 0;
  }

  /** \brief add frame and pixel coordinate
   * @param f  --- frame 
   */ 
  void addFrame(const Frame *f) {
    cv::Point2f pixel;
    if (f->getCornerUV(id,pixel)) {
      uv[f] = pixel;
    }
  }
  /** \brief remove frame ptr that corresponding this feature 
   * @param f  --- frame ptr that will be remove
   */ 
  void removeFrame(const Frame *f) {
    uv.erase(f);
  }

  /** \brief set this feature 3D position in the world
   * @param pts3D  --- feature 3D position in the world
   */ 
  void setPtsInWorld(const cv::Point3f& pts3D) {
    valid3D_ = true;
    p3D = pts3D;
  }

  /** \brief get feature coordinate in the world frame 
   * @return coordinate of features
   */ 
  cv::Point3f getPts3DInWorld() {
    return p3D;
  }

  /** \brief get this feature track count
   */ 
  int getTrackCount() const {
    return uv.size();
  }
  
  /** \brief get valid 3D
   * 
   */
  inline bool valid3D() const{
    return valid3D_;
  }

  /** \brief get features' map
   * @return return features' map
   */ 
  const std::map<const Frame*,cv::Point2f> &getFeatMap() const {
    return uv;
  }

  /** \brief judge frame has this point
   * @return return true if the frame has this feat
   */ 
  inline bool isInFrame(const Frame* f) {
    return (uv.count(f));
  }

  /** \brief increase bad Count
   */ 
  void incBadCount() {
    badCount_++;
  }

  /** \brief get badCount 
   */ 
  int getBadCount() const {
    return badCount_;
  }
 private:
  uint64 id;
  std::map<const Frame*,cv::Point2f> uv;
  cv::Point3f p3D;
  int badCount_;
  bool valid3D_;
};

class FeatureManager {
 public:

  /** \brief construct 
   */ 
  FeatureManager(){};

  /** \brief unconstruct function
   */  
  ~FeatureManager() {
    reset();
  };

  /** \brief remove feature with it's id
   */ 
  void removeFeature(uint64 id) {
    feats_.erase(id);
  }

  /** \brief remove feature with it's iterator
   */ 
  void removeFeature(std::map<uint64,Feature>::iterator it) {
    feats_.erase(it);
  }
  /** \brief remove fram from feature pixel map
   * @param  f   --- frame ptr
   */ 
  void removeFrame(Frame* f); //remove infomation about f in the feats_;

  /** \brief triangulate points in the frame which is not yet triangulated and tracked more than 3 times 
   * @param id    --- feature id
   * @param pts3d --- 3D coordinate in world frame
   * @return if success return true otherwise return false
   */ 
  bool triangulate(const Camera *cam,uint64 id,cv::Point3f &pt3d); //check every points in feats_ that no triangulate and track count more than track_cnt, then triangulate them 
  
  /** \brief get features in the FM which matching feature in the input frame
   * @param f      --- input frame f
   * @param curUV  --- matched feature' pixel coordinates in input frame f
   * @param matched3DPts  ---  matched feature 3D coordinate in FM
   */ 
  void featureMatching(const Camera *cam,const Frame* f,std::vector<uint64_t>& matchedIds,std::vector<cv::Point2f>& matchedNormalizedUV,std::vector<cv::Point3f>& matched3DPts);

  /** \brief add feature in the FM
   * @param id  ---  id of feature
   * @param f   --- parent frame of feature
   */ 
  inline void addFeature(uint64 id,Frame* f) {
    if (feats_.count(id)) {
      feats_[id].addFrame(f);
    } else {
      Feature feature(id,f);
      feats_[id] = feature;
    }
  }
  
  /** \brief update bad count of id feature
   */
  void updateBadCount(uint64_t id) {
    if (feats_.count(id)) {
      feats_[id].incBadCount();
    }
  } 

  /** \brief set feature 3D coordinate in the world frame
   * @param id    ---  id of the feature
   * @param pts3D ---  3D coordinate 
   */
  inline void setFeatPts3D(uint64 id,cv::Point3f& pts3D) {
    if (feats_.count(id)) {
      feats_[id].setPtsInWorld(pts3D);
    }
  }
  /** \brief get feature map size
   * 
   */ 
  inline int getFeatureSize() const{
    return feats_.size();
  }

  /** \brief get features' map
   */ 
  std::map<uint64,Feature>& getFeatureMap() {
    return feats_;
  }

  /** \brief get features coordinates in the world  
   */ 
  std::vector<cv::Vec3f> getPointsInWorld() {
    std::vector<cv::Vec3f> pts3D;
    for (auto p : feats_) {
      if (p.second.valid3D()) {
        cv::Point3f p3D = p.second.getPts3DInWorld();
        pts3D.push_back(cv::Vec3f(p3D.x,p3D.y,p3D.z));
      }
    }
    return pts3D;
  }

  /** \brief clear features' map
   */ 
  inline void reset() {
    feats_.clear();
  }

 private:
  std::map<uint64,Feature> feats_;
};
}