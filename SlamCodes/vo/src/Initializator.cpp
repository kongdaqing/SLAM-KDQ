#include "Initializator.hpp"

namespace ov {

Initializator::Initializator(const Config* cfg,Camera* cam):
    MinDisparity(cfg->iniParam_.minDisparity),
    InitialMinMatchedPointNum(cfg->iniParam_.minMatchedFeatNum),
    InitialReprojectErr(cfg->iniParam_.reprojectErr),
    HomographyTransformErr(cfg->iniParam_.homographyTransformErr),
    camera_(cam) {

};

bool Initializator::initPoseAndMap(Frame* refFrame,Frame* curFrame,FeatureManager& fs) {
  if (refFrame == nullptr || curFrame == nullptr) {
    printf("[Initializator]:No enough frames in the slidewindow!\n");
    return false;
  }
  if (fs.getFeatureSize() > 0) {
    printf("[Initializator]:Please clear feature maps firstly,then intializing!\n");
    return false;
  }
  std::map<uint64,cv::Point3f> pts3D;
  //step-1: 计算单应矩阵并解算当前帧和参考帧的位姿，同时三角化特征点
  if (!initializeFromHomography(refFrame,curFrame,pts3D)) {
    return false;
  }
  //step-2: 把参考帧中三角化成功的特征点加入到fsm中
  for (auto it = pts3D.begin(); it != pts3D.end(); it++) {
    fs.addFeature(it->first,refFrame);
    fs.setFeatPts3D(it->first,it->second);
  } 
  //step-3: 把当前帧所有特征都添加到fsm中
  std::map<uint64,cv::Point2f> curCorners = curFrame->getCorners();
  for (auto it = curCorners.begin(); it != curCorners.end(); it++) {
    fs.addFeature(it->first,curFrame);
  }
  return true;
}

inline bool Initializator::checkHomography(const cv::Mat &H) {
  const double det = H.at<double>(0, 0) * H.at<double>(1, 1) - H.at<double>(1, 0) * H.at<double>(0, 1);
  if (det < 0) 
    return false;
  const double N1 = sqrt(H.at<double>(0, 0) * H.at<double>(0, 0) + H.at<double>(1, 0) * H.at<double>(1, 0));
  if (N1 > 4 || N1 < 0.1) 
    return false;
  const double N2 = sqrt(H.at<double>(0, 1) * H.at<double>(0, 1) + H.at<double>(1, 1) * H.at<double>(1, 1));
  if (N2 > 4 || N2 < 0.1)
    return false;
  const double N3 = sqrt(H.at<double>(2, 0) * H.at<double>(2, 0) + H.at<double>(2, 1) * H.at<double>(2, 1)); 
    return N3 <= 0.002;
}


bool Initializator::checkCornerDisparities(std::vector<cv::Point2f>& refCorners,std::vector<cv::Point2f>& curCorners) {
  assert(refCorners.size() == curCorners.size());
  std::vector<float> disparities;
  const int size = refCorners.size();
  disparities.clear();
  for (size_t i = 0; i < size; i++) {
    disparities.push_back(cv::norm(refCorners[i] - curCorners[i]));
  }
  std::sort(disparities.begin(),disparities.end());
  if (disparities[size/2] < MinDisparity) {
    printf("[Disparity-Check]:Failed! Middle Disparity is %3.2f less than threshold %3.2f!\n",disparities[size/2],MinDisparity);
    return false;
  } 
  return true;
}



bool Initializator::initializeFromHomography(Frame* refFrame,Frame* curFrame,std::map<uint64,cv::Point3f>& pts3D) {

  cv::Mat H;
  cv::Mat K = camera_->K();
  std::vector<uchar> inliers;
  std::vector<uint64> idVec;
  std::vector<cv::Point2f> refFeatures,curFeatures;
  curFrame->getMatchedFeatures(refFrame,idVec,refFeatures,curFeatures);
  if (refFeatures.size() < InitialMinMatchedPointNum || !checkCornerDisparities(refFeatures,curFeatures)) {
    return false;
  }
  //KDQ: homography initialization
  if (!calHomography(H,inliers,K,refFeatures,curFeatures)) {
    return false;
  }
  std::vector<cv::Mat> R,t,n;
  //Note:这里虽然特征点是float的型的，但是计算出来的R,t,n全是double类型的mat
  //Note:这里R,t的含义是参考帧到当前帧的转换Tc1c0,而非Tc0c1
  cv::decomposeHomographyMat(H,K,R,t,n);
  std::vector< std::map<uint64,cv::Point3f> > ptsInWorld;
  int bestId = checkRt(ptsInWorld,R,t,n,K,inliers,idVec,refFeatures,curFeatures);
  if (bestId < 0) {
    std::cout << "[CheckRT]:Can't find best pose" << std::endl;
    return false;
  } 
  pts3D = ptsInWorld[bestId];
  curFrame->setPoseInWorld(R[bestId].t(),-R[bestId].t() * t[bestId]);
  return true;
}

bool Initializator::calHomography(cv::Mat &H,
                                  std::vector<uchar> &inliers,
                                  const cv::Mat &K, 
                                  const std::vector<cv::Point2f> &refFeatures,
                                  const std::vector<cv::Point2f> &curFeatures) {
  if (refFeatures.size() < InitialMinMatchedPointNum) {
    printf("[Homography]:matched features is not enough!\n");
    return false;
  }
  H = cv::findHomography(refFeatures, curFeatures);
  if (!checkHomography(H)) { 
    printf("[Homography]:check homography self failed!\n"); 
    return false;
  }
  cv::Mat norm_H = K.inv() * H * K;
  cv::Mat w,U,V;
  cv::SVD::compute(norm_H, w,U,V);
  double d1 = w.at<double>(0);
  double d2 = w.at<double>(1);
  double d3 = w.at<double>(2);
  if (d1 / d2 < 1.00001 || d2 / d3 < 1.00001) {
    printf("[Homography]:Warning!Singular-value d1 d2 d3 too similar！");
    return false;
  }
  std::vector<cv::Point2f> ref2curFeats;
  int goodProjectSize = 0;
  cv::perspectiveTransform(refFeatures,ref2curFeats,H);
  inliers.clear();
  for (size_t i = 0; i < curFeatures.size(); i++ ) {
    if (cv::norm(ref2curFeats[i] - curFeatures[i]) < HomographyTransformErr) {
      goodProjectSize++;
      inliers.push_back(1);
    } else {
      inliers.push_back(0);
    }
  }
  if (goodProjectSize < refFeatures.size() * 0.75) {
    printf("[Homography-Check]:Warning!Good reprojected point size %d is not enough in all size=%d!\n",goodProjectSize,static_cast<int>(refFeatures.size()));
    return false;
  }
  return true;
}

int Initializator::checkRt(std::vector< std::map<uint64,cv::Point3f> > &ptsInWorld,
                        const std::vector<cv::Mat> &R,
                        const std::vector<cv::Mat> &t,
                        const std::vector<cv::Mat> &n,
                        const cv::Mat &K,
                        const std::vector<uchar> &inliers,
                        const std::vector<uint64> &idVec,
                        const std::vector<cv::Point2f> &refFeatures,
                        const std::vector<cv::Point2f> &curFeatures) {
  const size_t num = R.size();
  assert(num > 0);
  int maxCountIndex = 0;
  int maxCount = 0;
  int goodPtsCount[num];
  memset(goodPtsCount,0,sizeof(goodPtsCount));
  float sumDistance[num];
  memset(sumDistance,0.,sizeof(sumDistance));
  cv::Mat KF;
  K.convertTo(KF,CV_32F);
  ptsInWorld.clear();
  ptsInWorld.resize(num);
  assert(refFeatures.size() == inliers.size());
  assert(inliers.size() == idVec.size());
  std::map<int,cv::Point3f> localMap[num];
  for (size_t i = 0; i < num; i++) {
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    P1.colRange(0,3) = cv::Mat::eye(3,3,CV_32F);
    cv::Mat P2(3,4,CV_32F,cv::Scalar(0));
    cv::Mat Rs;
    cv::Mat ts;
    //Note：一定要注意，因为后续三角化特征点时候全采用float类型，因此这里需要用covertTo，而不能用copyto，因为copyto将把目标类型也拷贝成源类型
    //比如：R[i].copyTo(Rs),Rs变成了CV_64F
    R[i].convertTo(Rs,CV_32F);
    t[i].convertTo(ts,CV_32F);    
    Rs.copyTo(P2.colRange(0,3));
    ts.copyTo(P2.col(3));
    for (size_t j = 0; j < refFeatures.size(); j ++) {
      if (inliers[j] == 0) {
        continue;
      }
      cv::Point3f pts3d;
      cv::Point2f refUndistPt = camera_->normalized(refFeatures[j]);
      cv::Point2f curUndistPt = camera_->normalized(curFeatures[j]);
      triangulate(refUndistPt,curUndistPt,P1,P2,pts3d);
      //Note:如果c1P3D声明为double型,就不能用c1P3D.at<float>,这样将得到一个错误的值
      cv::Mat c1P3D = (cv::Mat_<double>(3,1) << pts3d.x,pts3d.y,pts3d.z);
      if (c1P3D.at<double>(2) < 0.1) {
        continue;
      }
      cv::Mat crw0 = (cv::Mat_<double>(3,1) << 0.,0.,0.);
      cv::Mat CtW0 = crw0;
      cv::Point2f uv0 = camera_->project(pts3d,crw0,CtW0);
      if (cv::norm(refFeatures[j] - uv0) > InitialReprojectErr) {
        continue;
      }
      cv::Mat crw1;
      cv::Rodrigues(R[i],crw1);
      //Note:因为旋转矩阵直接使用了double型的,所以必须也用double型的c1P3D,保持一致
      cv::Mat c2P3D = R[i] * c1P3D + t[i]; 
      if (c2P3D.at<double>(2) < 0.1) {
        continue;
      }
      cv::Point2f uv1 = camera_->project(pts3d,crw1,t[i]);
      if (cv::norm(curFeatures[j] - uv1) > InitialReprojectErr) {
        continue;
      }
      sumDistance[i] += cv::norm(refFeatures[j] - uv0) + cv::norm(curFeatures[j] - uv1);
      goodPtsCount[i]++;
      ptsInWorld[i][idVec[j]] = pts3d;
    }
    if (maxCount < goodPtsCount[i]) {
      maxCountIndex = i;
      maxCount = goodPtsCount[i];
    }
  }
  int bestIndex = maxCountIndex;
  if (maxCount < refFeatures.size() * 0.75) {
    bestIndex = -1;
  } else {
    //Note:有可能次优的解和真值具有相同的goodPtsCount,但是真值的averDist一定比次有解要小
    float miniDist = 100.;
    for (size_t i = 0; i < num; i++) {
      if (goodPtsCount[i] > 0.9 * maxCount && miniDist > sumDistance[i]/goodPtsCount[i]) {
        miniDist = sumDistance[i]/goodPtsCount[i];
        bestIndex = i;
        std::cout << i  << " Count = " << goodPtsCount[i] 
            << " n = " << n[i].t() 
            << " averDist = " << sumDistance[i]/goodPtsCount[i] << std::endl;
      }
    }
  }
  return bestIndex;
}


/** 给定投影矩阵P1,P2和图像上的匹配特征点点kp1,kp2，从而计算三维点坐标
 * @brief 
 * 
 * @param[in] kp1               特征点, in reference frame
 * @param[in] kp2               特征点, in current frame
 * @param[in] P1                pose from world to reference frame
 * @param[in] P2                pose from world to current frame
 * @param[in & out] x3D         计算的三维点
 */
 void Initializator::triangulate(const cv::Point2f &kp1,    //特征点, in reference frame
                             const cv::Point2f &kp2,    //特征点, in current frame
                             const cv::Mat &P1,          //投影矩阵P1
                             const cv::Mat &P2,          //投影矩阵P2
                             cv::Point3f &pts3d)  {
    // 原理
    // Trianularization: 已知匹配特征点对{x x'} 和 各自相机矩阵{P P'}, 估计三维点 X
    // x' = P'X  x = PX
    // 它们都属于 x = aPX模型
    //                         |X|
    // |x|     |p1 p2  p3  p4 ||Y|     |x|    |--p0--||.|
    // |y| = a |p5 p6  p7  p8 ||Z| ===>|y| = a|--p1--||X|
    // |z|     |p9 p10 p11 p12||1|     |z|    |--p2--||.|
    // 采用DLT的方法：x叉乘PX = 0
    // |yp2 -  p1|     |0|
    // |p0 -  xp2| X = |0|
    // |xp1 - yp0|     |0|
    // 两个点:
    // |yp2   -  p1  |     |0|
    // |p0    -  xp2 | X = |0| ===> AX = 0
    // |y'p2' -  p1' |     |0|
    // |p0'   - x'p2'|     |0|
    // 变成程序中的形式：
    // |xp2  - p0 |     |0|
    // |yp2  - p1 | X = |0| ===> AX = 0
    // |x'p2'- p0'|     |0|
    // |y'p2'- p1'|     |0|
    // 然后就组成了一个四元一次正定方程组，SVD求解，右奇异矩阵的最后一行就是最终的解.

	//这个就是上面注释中的矩阵A
    cv::Mat A(4,4,CV_32F);

	//构造参数矩阵A
    A.row(0) = kp1.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.y*P2.row(2)-P2.row(1);

	//奇异值分解的结果
    cv::Mat u,w,vt;
	//对系数矩阵A进行奇异值分解
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
	//根据前面的结论，奇异值分解右矩阵的最后一行其实就是解，原理类似于前面的求最小二乘解，四个未知数四个方程正好正定
	// //别忘了我们更习惯用列向量来表示一个点的空间坐标
  //   x3D = vt.row(3).t();
	// //为了符合其次坐标的形式，使最后一维为1
    cv::Mat x3D = vt.row(3).t();  
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3); 
    pts3d.x = x3D.at<float>(0);
    pts3d.y = x3D.at<float>(1);
    pts3d.z = x3D.at<float>(2);
}

}
