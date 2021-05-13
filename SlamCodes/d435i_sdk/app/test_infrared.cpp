#include "d435i.hpp"

int main(int argc,char **argv) {
  D435I d435i;
  d435i.start();
  while(1) {
    cv::Mat leftImg,rightImg;
    d435i.getInfraredImages(leftImg,rightImg);
    if (!leftImg.empty() && !rightImg.empty()) {
      cv::Mat img;
      cv::hconcat(leftImg,rightImg,img);
      cv::imshow("infraredImages",img);
      cv::waitKey(1);
    }
    cv::waitKey(1);
  }
  return 0;
}