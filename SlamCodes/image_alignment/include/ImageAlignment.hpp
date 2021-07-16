#include "iostream"
#include "opencv2/opencv.hpp"

class ImageAlignment {
 public:
  ImageAlignment(int motionModel,int iterationNum,double eps,int pydownLevel = 1,bool histgram = false):
    MotionModel_(motionModel),
    IterationNumber_(iterationNum),
    TerminationEps_(eps),
    Histgram_(histgram),
    PyrDownLevel_(pydownLevel){
    if (PyrDownLevel_ < 1) {
      PyrDownLevel_ = 1;
    }
  }
  cv::Mat Align(cv::Mat& templateIm,cv::Mat& targetIm) {
    cv::Mat img1,img2;
    cv::pyrDown(templateIm,img1,templateIm.size()/PyrDownLevel_);
    cv::pyrDown(targetIm,img2,templateIm.size()/PyrDownLevel_);
    cv::Mat warpMatrix;
    if (MotionModel_ > cv::MOTION_HOMOGRAPHY || MotionModel_ < cv::MOTION_TRANSLATION) {
      MotionModel_ = cv::MOTION_HOMOGRAPHY;
    }
    if (MotionModel_ == cv::MOTION_HOMOGRAPHY) {
      warpMatrix = cv::Mat::eye(3, 3, CV_32F);
    } else {
      warpMatrix = cv::Mat::eye(2, 3, CV_32F);
    }
    cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, IterationNumber_, TerminationEps_);
    cv::Mat im1 = img1.clone(),im2 = img2.clone();
    if (templateIm.channels() == 3) {
      cv::cvtColor(templateIm,im1,CV_BGR2GRAY);
    }
    if (targetIm.channels() == 3) {
      cv::cvtColor(targetIm,im2,CV_BGR2GRAY);
    }
    if (Histgram_) {
      cv::equalizeHist(im1,im1);
      cv::equalizeHist(im2,im2);
    }
    cv::findTransformECC(im1,im2,warpMatrix,MotionModel_,termCriteria);
#ifdef PC_VERSION
    cv::Mat im2_aligned;
    if (MotionModel_ != cv::MOTION_HOMOGRAPHY) {
      // Use warpAffine for Translation, Euclidean and Affine
      cv::warpAffine(im2, im2_aligned, warpMatrix, im1.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    } else {
      // Use warpPerspective for Homography
      cv::warpPerspective(im2, im2_aligned, warpMatrix, im1.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    }
    cv::putText(im1,"im1",cv::Point2f(20,10),cv::FONT_HERSHEY_COMPLEX_SMALL,1.0,cv::Scalar(0,255,0),1);
    cv::putText(im2,"im2",cv::Point2f(20,10),cv::FONT_HERSHEY_COMPLEX_SMALL,1.0,cv::Scalar(0,255,0),1);
    cv::putText(im2_aligned,"im2_aligned",cv::Point2f(20,10),cv::FONT_HERSHEY_COMPLEX_SMALL,1.0,cv::Scalar(0,255,0),1);
    cv::Mat disIm;
    cv::hconcat(im1,im2_aligned,disIm);
    cv::hconcat(disIm,im2,disIm);
    // Show final result
    imshow("ImageAlignment", disIm);
    cv::waitKey(1);
#endif
    return warpMatrix;
  }
 private:
  bool Histgram_;
  int MotionModel_;
  int IterationNumber_;
  double TerminationEps_;
  int PyrDownLevel_;
};