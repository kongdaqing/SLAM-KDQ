#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

class OptiflowTrackByVINS {
  private:
    cv::Mat prev_img_,cur_img_,track_img_,mask_;
    std::vector<cv::Point2f> prev_pts_, cur_pts_;
    std::vector<int> ids_,track_cnt_;
    int id_,track_size_;
    double t_,track_cost_,detect_cost_;
    float fastResponseThreshold_;
    int featureSize_,cols_,rows_;
    bool trackBack_;
    float min_dist_;
    std::string save_image_path_;
    void addNewPoints();
    void setMask();
    void showTrack();
    double distance(const cv::Point2f& pt1,const cv::Point2f& pt2);
    bool inBorder(const cv::Point2f& pt);
    void drawTrack();
    template <typename RType>
    static void reduceVector(const std::vector<uchar>& status,std::vector<RType>& vec);

  public:
    OptiflowTrackByVINS(int featSize,float thres,float minDist,std::string savePath,bool trackBack);
    ~OptiflowTrackByVINS();

    std::vector<cv::Point2f> optiTrackFeature(double timestamp,const cv::Mat& img);
    
};