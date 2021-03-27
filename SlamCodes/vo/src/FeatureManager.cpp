#include "FeatureManager.hpp"
namespace ov {

void FeatureManager::removeFrame(Frame *f) {
  for(auto it = feats_.begin(); it != feats_.end(); it++) {
    it->second.removeFrame(f);
  }
}
bool FeatureManager::triangulate(uint64 id,cv::Point3f &pt3d) {
  if (!feats_.count(id) || feats_[id].getTrackCount() < 2) {
    return false;
  }
  const std::map<const Frame*,cv::Point2f>& uv = feats_[id].getFeatMap();
  cv::Mat A(2 * feats_[id].getTrackCount(),4,CV_32F);
  int i = 0;
  for (auto it = uv.begin(); it != uv.begin(); it++) {
    cv::Mat R,t;
    it->first->getPoseInWorld(R,t);
    cv::Mat P(3,4,CV_32F);
    R.copyTo(P.colRange(0,3));
    t.copyTo(P.col(3));
    cv::Point2f kp = camera_->normalized(it->second);
    A.row(i) = kp.x*P.row(2) - P.row(0);
    A.row(i+1) = kp.y*P.row(2) - P.row(1);
    i +=2;
  }
  cv::Mat u,w,vt,x3D;
  cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
  x3D = vt.row(3).t();
  x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
  if (x3D.at<float>(2) < 0.1) {
    return false;
  }
  pt3d.x = x3D.at<float>(0);
  pt3d.y = x3D.at<float>(1);
  pt3d.z = x3D.at<float>(2);
  return true;
}

void FeatureManager::featureMatching(const Frame *f,std::vector<cv::Point2f>& matchedNormalizedUV,std::vector<cv::Point3f>& matched3DPts) {
  const std::map<uint64,cv::Point2f> &corners = f->getCorners();
  std::vector<std::pair<int,uint64>> candiFeats;
  matchedNormalizedUV.clear();
  matched3DPts.clear();
  for (auto it = corners.begin(); it != corners.end(); it++) {
    const uint64 id = it->first;
    if (feats_.count(id)) {
      //Condition1: 如果该特征点已经三角化了，可以参与Pnp求解
      if (feats_[id].valid3D()) {
        cv::Point2f normlizedUV = camera_->normalized(it->second);
        matchedNormalizedUV.push_back(normlizedUV);
        matched3DPts.push_back(feats_[id].getPts3DInWorld());
      } else if (feats_[id].getTrackCount() > 4) {
      //Condition2: 如果该点track次数超过4次，但是没有被三角化，那么可以直接三角化  
        cv::Point3f pt3d;
        if (triangulate(id,pt3d)) {
          feats_[id].setPtsInWorld(pt3d);
        }
        if(feats_[id].valid3D()) {
          cv::Point2f normlizedUV = camera_->normalized(it->second);
          matchedNormalizedUV.push_back(normlizedUV);
          matched3DPts.push_back(feats_[id].getPts3DInWorld());
        }
      } else if (feats_[id].getTrackCount() >= 2) {
      //Condition3: 如果该点track次数已经有2次或者以上，可以先作为候选点  
        candiFeats.push_back(std::make_pair(feats_[id].getTrackCount(),id));
      }
    }
  }

  if (matchedNormalizedUV.size() < 30 && !candiFeats.empty()) {
    auto cmp = [](std::pair<int,uint64>& a,std::pair<int,uint64>& b) {
      return a.first > b.first? true:false;
    };
    std::sort(candiFeats.begin(),candiFeats.end(),cmp);
    std::vector<std::pair<int,uint64>>::const_iterator it = candiFeats.begin();
    while (it != candiFeats.end() && matchedNormalizedUV.size() < 30) {
      cv::Point3f pt3d;
      if (triangulate(it->second,pt3d)) {
        feats_[it->second].setPtsInWorld(pt3d);
      }
      if (feats_[it->second].valid3D()) {
        cv::Point2f normlizedUV = camera_->normalized(corners.at(it->second));
        matchedNormalizedUV.push_back(normlizedUV);
        matched3DPts.push_back(feats_[it->second].getPts3DInWorld()); 
      }
      it++;
    }
  }
}


}