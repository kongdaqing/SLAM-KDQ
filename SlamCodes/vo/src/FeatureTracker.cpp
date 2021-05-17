#include "FeatureTracker.hpp"
namespace ov {

FeatureTracker::FeatureTracker(const Config* cfg):
    MaxPointSize(cfg->optParam_.maxPoints),
    QualityLevel(cfg->optParam_.qualityLevel),
    MinDist(cfg->optParam_.minDist),
    TrackBack(cfg->optParam_.trackBack),
    PyramidLevel(cfg->optParam_.pyramidLevel),
    CriterIterations(cfg->optParam_.iterations),
    CriterEPS(cfg->optParam_.eps),
    ShowTrackFrames(cfg->optParam_.showTrackFrames) {
  cam_ = new Camera(cfg);
  reset();
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

  if (ShowTrackFrames > 0) {
    std::map<uint64,cv::Point2f> curFeats = curFrame->getCornersCopy();
    allCorners_.push_back(curFeats);
    if (allCorners_.size() > ShowTrackFrames) {
      allCorners_.pop_front();
    }
    showAllFeature(curFrame->image_);
  }

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


void FeatureTracker::showAllFeature(cv::Mat &img,uint8_t imgScale) {
  cv::Mat colorImg;
  cv::cvtColor(img,colorImg,cv::COLOR_GRAY2BGR);
  std::map<uint64,cv::Point2f> cornerFront = allCorners_.front();
  std::map<uint64,cv::Point2f> cornerBack = allCorners_.back();
  for(auto it = cornerBack.begin(); it != cornerBack.end(); it++) {
    uint64 id = it->first;
    if (cornerFront.count(id)) {
      cv::Point2f pointBegin = cornerFront[id];
      cv::Point2f pointEnd = cornerBack[id];
      cv::putText(colorImg,std::to_string(it->first),pointEnd,cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255));
      cv::line(colorImg,pointBegin,pointEnd,cv::Scalar(0,255,0));
    }
  }
  cv::resize(colorImg,colorImg,cv::Size(colorImg.cols * imgScale, colorImg.rows * imgScale));
  cv::imshow("all features",colorImg);
  cv::waitKey(1);
}

}