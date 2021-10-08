#include <iostream>
#include "ImageGrayValueHistogram.hpp"

int main(int argc,char** argv) {
  if (argc != 2) {
    std::cout << "Please input one image" << std::endl;
    return -1;
  }
  std::string imageFile = argv[1];
  cv::Mat image = cv::imread(imageFile,cv::IMREAD_GRAYSCALE);
  ImageGrayValueHistogram gryCalHist(3,3,4);
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
  cv::imshow("image",image);
  while(cv::waitKey(0) != 'q') {

  }
}