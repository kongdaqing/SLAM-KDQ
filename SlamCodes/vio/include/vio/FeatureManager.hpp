#pragma once 
#include "Frame.hpp"
#include "Camera.hpp"
#define WINSIZE 10
namespace vio{

typedef std::pair<cv::Point2f,cv::Point2f> PixelCoordinate;

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
  Feature(uint64_t idx,FramePtr f) {
    id = idx;
    addFrame(f);
    valid3D_ = false;
    badCount_ = 0;
    goodCount_ = 0;
  }

  /** \brief add frame and pixel coordinate
   * @param f  --- frame 
   */ 
  void addFrame(const FramePtr f) {
    cv::Point2f pixel,normalized;
    if (f->getCornerUV(id,pixel) && f->getNormalizedUV(id,normalized)) {
      uv[f] = PixelCoordinate(pixel,normalized);
    }
  }
  /** \brief remove frame ptr that corresponding this feature 
   * @param f  --- frame ptr that will be remove
   */ 
  void removeFrame(const FramePtr f) {
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
  cv::Point3f getPts3DInWorld() const {
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
  const std::map<const FramePtr,PixelCoordinate> &getFeatMap() const {
    return uv;
  }

  /** \brief judge frame has this point
   * @return return true if the frame has this feat
   */ 
  inline bool isInFrame(const FramePtr& f) {
    return (uv.count(f));
  }

  bool getPixelInFrame(const FramePtr& f,PixelCoordinate& pixel) {
    if (uv.count(f)) {
      pixel = uv[f];
      return true;
    }
    return false;
  }

  /** \brief increase bad Count
   */ 
  void incBadCount() {
    badCount_++;
  }

  void incGoodCount() {
    goodCount_++;
  }

  /** \brief get badCount 
   */ 
  int getBadCount() const {
    return badCount_;
  }

  int getGoodCount() const {
    return goodCount_;
  }

  bool isReadyForOptimize() const{
    return valid3D_ && badCount_ == 0 && getTrackCount() > 1;
  }
 private:
  uint64_t id;
  std::map<const FramePtr,PixelCoordinate> uv;
  cv::Point3f p3D;
  int badCount_;
  int goodCount_;
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
  void removeFeature(uint64_t id) {
    feats_.erase(id);
  }

  /** \brief remove feature with it's iterator
   */ 
  void removeFeature(std::map<uint64_t,Feature>::iterator it) {
    feats_.erase(it);
  }
  /** \brief remove fram from feature pixel map
   * @param  f   --- frame ptr
   */ 
  void removeFrame(FramePtr f); //remove infomation about f in the feats_;

  /** \brief triangulate points in the frame which is not yet triangulated and tracked more than 3 times 
   * @param id    --- feature id
   * @param pts3d --- 3D coordinate in world frame
   * @return if success return true otherwise return false
   */ 
  bool triangulate(uint64_t id,cv::Point3f &pt3d); //check every points in feats_ that no triangulate and track count more than track_cnt, then triangulate them


  void triangulateAll(); //check every points in feats_ that no triangulate and track count more than track_cnt, then triangulate them


  /** \brief get features in the FM which matching feature in the input frame
   * @param f      --- input frame f
   * @param curUV  --- matched feature' pixel coordinates in input frame f
   * @param matched3DPts  ---  matched feature 3D coordinate in FM
   */ 
  void featureMatching(const FramePtr f,std::vector<uint64_t>& matchedIds,std::vector<cv::Point2f>& matchedNormalizedUV,std::vector<cv::Point3f>& matched3DPts);

  /** \brief insert frame
   * @param f --- frame ptr
   */
  void addFrame(FramePtr f) {
    std::map<uint64_t,cv::Point2f> &corners = f->getCorners();
    for (auto it = corners.begin();it != corners.end(); it++) {
      addFeature(it->first,f);
    }
  }
  /** \brief add feature in the FM
   * @param id  ---  id of feature
   * @param f   --- parent frame of feature
   */ 
  inline void addFeature(uint64_t id,FramePtr f) {
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

  void updateGoodCount(uint64_t id) {
    if (feats_.count(id)) {
      feats_[id].incGoodCount();
    }
  }
  /** \brief set feature 3D coordinate in the world frame
   * @param id    ---  id of the feature
   * @param pts3D ---  3D coordinate
   * @param coverFlg --- if feature 3d points is valid,cover value when coverflg == ture else reject value.
   */
  inline void setFeatPts3D(uint64_t id,cv::Point3f& pts3D,bool coverFlg = true) {
    if (feats_.count(id)) {
      if (!feats_[id].valid3D()) {
        feats_[id].setPtsInWorld(pts3D);
      } else {
        if (coverFlg) {
          feats_[id].setPtsInWorld(pts3D);
        }
      }
    } else {
      printf("[FSM]:set feature 3D position failed!%lu corner doesn't exsit!\n",id);
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
  std::map<uint64_t,Feature>& getFeatureMap() {
    return feats_;
  }

  const std::map<uint64_t,Feature>& getFeatureMap() const {
    return feats_;
  }

  /** \brief get features coordinates in the world  
   */ 
  std::vector<cv::Vec3f> getPointsInWorld() const{
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
  std::map<uint64_t ,Feature> feats_;
};
}