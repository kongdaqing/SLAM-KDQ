#include "FeatureTracker.hpp"
namespace ov {

FeatureTracker::FeatureTracker(const Config* cfg):
    MaxPointSize(cfg->optParam_.maxPoints),
    QualityLevel(cfg->optParam_.qualityLevel),
    MinDist(cfg->optParam_.minDist),
    TrackBack(cfg->optParam_.trackBack),
    PyramidLevel(cfg->optParam_.pyramidLevel),
    CriterIterations(cfg->optParam_.iterations),
    CriterEPS(cfg->optParam_.eps) {
  id_ = 0;
  cam_ = new Camera(cfg);
  trackCount_.clear();
};



void FeatureTracker::detectAndTrackFeature(FramePtr refFrame,FramePtr curFrame) {
  if (curFrame == nullptr || curFrame->image_.empty()) {
    return;
  }
  std::vector<uint64_t> idx;
  std::vector<cv::Point2f> refCorners,curCorners;
  if (refFrame != nullptr) {
    refFrame->getCornerVector(idx,refCorners);
    curCorners = refCorners; 
    if (refCorners.size() > 5) {
      std::vector<uchar> status;
      cv::Mat err;
      cv::calcOpticalFlowPyrLK(refFrame->image_,curFrame->image_,refCorners,curCorners,status,err,
                              cv::Size(21,21),PyramidLevel,cv::TermCriteria(1,CriterIterations,CriterEPS));
      for (size_t i = 0; i < status.size(); i++) {
        if (status[i] && !cam_->isInFrame(curCorners[i])) {
          status[i] = 0;
        } 
      }
      remove(status,refCorners);
      remove(status,curCorners);
      remove(status,idx);
      remove(status,trackCount_);
      
      if (TrackBack && curCorners.size() > 5) {
        std::vector<uchar> status;
        std::vector<cv::Point2f> refBackCorners = refCorners;
        cv::Mat err;
        cv::calcOpticalFlowPyrLK(curFrame->image_,refFrame->image_,curCorners,refBackCorners,status,err,
                                 cv::Size(21,21),1,cv::TermCriteria(1,10,0.1));
        for (size_t i = 0; i < status.size(); i++) {
          if (status[i] && (!cam_->isInFrame(refBackCorners[i]) || cv::norm(refCorners[i] - refBackCorners[i]) > 2.0)) {
            status[i] = 0;
          } 
        }
        remove(status,curCorners);
        remove(status,idx);
        remove(status,trackCount_);
      }
      for (size_t i = 0; i < trackCount_.size(); i++) {
        trackCount_[i]++;
      }
      
    }
  } 
  cv::Mat mask = setMask(curFrame->image_,idx,curCorners);
  curFrame->setCornerMap(idx,curCorners);
  const int needFeatSize = MaxPointSize - curCorners.size();
  if (needFeatSize > 0.25 * MaxPointSize) {
    std::vector<cv::Point2f> feats;
    cv::goodFeaturesToTrack(curFrame->image_,feats,needFeatSize,QualityLevel,MinDist,mask);
    std::vector<uint64> newIdx;
    for (size_t i = 0;i < feats.size(); i++) {
      id_++;
      newIdx.push_back(id_);
      trackCount_.push_back(1);
    }
    curFrame->setCornerMap(newIdx,feats);
  }
}

}