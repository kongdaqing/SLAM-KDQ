#include "FeatureTracker.hpp"
#include "tictoc.hpp"
namespace vio{

FeatureTracker::FeatureTracker(const Config* cfg):
    MaxPointSize(cfg->optParam_.maxPoints),
    QualityLevel(cfg->optParam_.qualityLevel),
    MinDist(cfg->optParam_.minDist),
    TrackBack(cfg->optParam_.trackBack),
    TrackBackPixelErr(cfg->optParam_.trackBackPixelErr),
    PyramidLevel(cfg->optParam_.pyramidLevel),
    CriterIterations(cfg->optParam_.iterations),
    CriterEPS(cfg->optParam_.eps),
    ShowTrackFrames(cfg->optParam_.showTrackFrames),
    ShowDebugInfos(cfg->optParam_.showDebugInfos) {
  cam_ = new Camera(cfg);
  reset();
};

std::vector<cv::Point2f> FeatureTracker::predictFeatures(const std::vector<cv::Point2f>& lastCorners,const cv::Mat& RcurLast) {
  if (RcurLast.empty()) {
    return lastCorners;
  }
  std::vector<cv::Point2f> curCorners;
  for(auto p : lastCorners) {
    cv::Point2f lastNormalized2D = cam_->normalized(p);
    cv::Mat lastNormalized3D  = (cv::Mat_<float>(3,1) << lastNormalized2D.x,lastNormalized2D.y,1.);
    cv::Mat curNormalized3D = RcurLast * lastNormalized3D;
    cv::Point3f curCorner3D;
    curCorner3D.x = curNormalized3D.at<float>(0) / curNormalized3D.at<float>(2);
    curCorner3D.y = curNormalized3D.at<float>(1) / curNormalized3D.at<float>(2);
    curCorner3D.z = 1.0;
    cv::Point2f curCorner2D = cam_->project(curCorner3D);
    if (cam_->isInFrame(curCorner2D)) {
      curCorners.push_back(curCorner2D);
    } else {
      curCorners.push_back(p);
    }
  }
  return curCorners;
}

void FeatureTracker::detectAndTrackFeature(FramePtr refFrame,FramePtr curFrame,const cv::Mat &RcurLast) {
  Tictoc tictoc("track"),allTicToc("allCost");
  double costTime[4] = {0.};
  allTicToc.tic();
  if (curFrame == nullptr || curFrame->image_.empty()) {
    return;
  }
  std::vector<uint64_t> idx;
  std::vector<cv::Point2f> refCorners,curCorners;
  if (refFrame != nullptr) {
    tictoc.tic();
    refFrame->getCornerVector(idx,refCorners);
    curCorners = predictFeatures(refCorners,RcurLast);
    if (ShowTrackFrames && !RcurLast.empty()) {
      for(auto p : curCorners) {
        cv::Mat showImg;
        cv::cvtColor(curFrame->image_,showImg,cv::COLOR_GRAY2BGR);
        cv::circle(showImg,p,2,cv::Scalar(0,0,255),2);
        cv::imshow("predict corners",showImg);
        cv::waitKey(1);
      }
    }
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
      costTime[0] = tictoc.toc();
      remove(status,refCorners);
      remove(status,curCorners);
      remove(status,idx);
      remove(status,trackCount_);
      tictoc.tic();
      if (TrackBack && curCorners.size() > 5) {
        std::vector<uchar> status;
        std::vector<cv::Point2f> refBackCorners = refCorners;
        cv::Mat err;
        cv::calcOpticalFlowPyrLK(curFrame->image_,refFrame->image_,curCorners,refBackCorners,status,err,
                                 cv::Size(21,21),1,cv::TermCriteria(1,10,0.1),1);
        for (size_t i = 0; i < status.size(); i++) {
          if (status[i] && (!cam_->isInFrame(refBackCorners[i]) || cv::norm(refCorners[i] - refBackCorners[i]) > TrackBackPixelErr)) {
            status[i] = 0;
          } 
        }
        costTime[1] = tictoc.toc();
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
  tictoc.tic();
  if (ShowTrackFrames > 0) {
    std::map<uint64,cv::Point2f> curFeats = curFrame->getCornersCopy();
    allCorners_.push_back(curFeats);
    if (allCorners_.size() > ShowTrackFrames) {
      allCorners_.pop_front();
    }
    showAllFeature(curFrame->image_);
  }
  costTime[2] = tictoc.toc();
  tictoc.tic();
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
  costTime[3] = tictoc.toc();
  double allCostMs = allTicToc.toc();
  if (ShowDebugInfos) {
    std::cout << "[OptiTrack]:   All-Modules Cost(ms): " << allCostMs   << "\n"
              << "             Forward-Track Cost(ms): " << costTime[0] << "\n"
              << "                Back-Track Cost(ms): " << costTime[1] << "\n"
              << "                    Imshow Cost(ms): " << costTime[2] << "\n"
              << "             Extract-Point Cost(ms): " << costTime[3] << "\n";

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