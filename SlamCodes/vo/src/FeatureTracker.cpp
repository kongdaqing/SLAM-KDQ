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
};



void FeatureTracker::detectAndTrackFeature(FramePtr refFrame,FramePtr curFrame) {
  if (refFrame != nullptr) {
    std::vector<uint64> idx;
    std::vector<cv::Point2f> refCorners,curCorners;
    refFrame->getCornerVector(idx,refCorners);
    curCorners = refCorners; 
    if (refCorners.size() > 5) {
      std::vector<uchar> status;
      cv::calcOpticalFlowPyrLK(refFrame->image_,curFrame->image_,refCorners,curCorners,status,cv::Mat(),
                              cv::Size(21,21),PyramidLevel,cv::TermCriteria(1,CriterIterations,CriterEPS));
      remove(status,refCorners);
      remove(status,curCorners);
      remove(status,idx);
      if (TrackBack) {
        std::vector<uchar> status;
        std::vector<cv::Point2f> refBackCorners = refCorners;
        cv::calcOpticalFlowPyrLK(curFrame->image_,refFrame->image_,curCorners,refBackCorners,status,cv::Mat(),
                                 cv::Size(21,21),1,cv::TermCriteria(1,10,0.1));
        for (size_t i = 0; i < status.size(); i++) {
          if (status[i] && cv::norm(refCorners[i] - refBackCorners[i]) > 2.0) {
            status[i] = 0;
          } 
        }
        remove(status,curCorners);
        remove(status,idx);
      }
      curFrame->setCornerMap(idx,curCorners);
    }
  } 

  std::vector<uint64> idx;
  std::vector<cv::Point2f> curCorners;
  curFrame->getCornerVector(idx,curCorners);
  cv::Mat mask = setMask(curFrame->image_,curCorners);
  const int needFeatSize = MaxPointSize - curCorners.size();
  if (needFeatSize > 0.25 * MaxPointSize) {
    std::vector<cv::Point2f> feats;
    cv::goodFeaturesToTrack(curFrame->image_,feats,needFeatSize,QualityLevel,MinDist,mask);
    std::vector<uint64> newIdx;
    for (size_t i = 0;i < feats.size(); i++) {
      id_++;
      newIdx.push_back(id_);
    }
    curFrame->setCornerMap(newIdx,feats);
  }
}

}