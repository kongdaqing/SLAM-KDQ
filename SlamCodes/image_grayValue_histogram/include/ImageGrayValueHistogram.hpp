//
// Created by kdq on 2021/10/8.
//
#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

class ImageGrayValueHistogram {
 public:
  ImageGrayValueHistogram() = delete;
  ImageGrayValueHistogram(int gridCols = 2,int gridRows = 2,int downSample = 1):
    gridCols_(gridCols),
    gridRows_(gridRows),
    downSample_(downSample) {
  }
  void CalculateGridsGrayValue(cv::Mat& image,std::vector<std::vector<float>>& gridValues) {
    if (image.empty()) {
      return;
    }
    cv::Mat img;
    cv::resize(image,img,cv::Size(image.cols/downSample_,image.rows/downSample_),0,0,CV_INTER_AREA);
    int cellHeight = floor(img.rows / gridRows_);
    int cellWidth = floor(img.cols / gridCols_);
    gridValues.resize(gridRows_,std::vector<float>(gridCols_));
    for (int i = 0; i < gridRows_; ++i) {
      for (int j = 0; j < gridCols_; ++j) {
        int x = j * cellWidth;
        int y = i * cellHeight;
        cv::Mat tmp(img,cv::Rect(x,y,cellWidth,cellHeight));
        gridValues[i][j] = cv::mean(tmp)[0];
      }
    }
  }
 private:
  const int gridCols_;
  const int gridRows_;
  const int downSample_;
};