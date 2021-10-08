#include <iostream>
#include "ImageGrayValueHistogram.hpp"

int main(int argc,char** argv) {
  cv::Mat image = cv::Mat(400,800,CV_8UC1,cv::Scalar(0));

  for (int y = 0; y < 400; ++y) {
    for (int x = 0; x < 800; ++x) {
      int sum = (y / 200) * 4 + x / 200;
      image.at<uchar>(y,x) = sum * 25;
    }
  }
  ImageGrayValueHistogram gryCalHist(4,2,1);
  std::vector<std::vector<float>> histogram;
  gryCalHist.CalculateGridsGrayValue(image,histogram);
  for (int i = 0; i < histogram.size(); ++i) {
    int j = 0;
    for (auto v: histogram[i]) {
      printf("<%d,%d>:%f, ", i,j,v);
      j++;
    }
    std::cout << std::endl;
  }
  cv::imshow("test",image);

  while(cv::waitKey(30) != 'q' ) {

  }
}