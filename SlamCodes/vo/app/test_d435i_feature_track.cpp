#include "d435i.hpp"
#include "FeatureTracker.hpp"

int main() {
  D435I d435i_device;
  d435i_device.start();
  while (1) {
    cv::Mat leftInfrared1,rightInfrared2;
    d435i_device.getInfraredImages(leftInfrared1,rightInfrared2);
  }
}
