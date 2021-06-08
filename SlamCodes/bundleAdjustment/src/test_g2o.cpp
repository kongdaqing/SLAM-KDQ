#include <iostream>
#include "BundleAdjustmentByG2O.hpp"
int main() {
  BundleAdjustmentByG2O ba;
  ba.testTcw();
  std::cout << "===============================================================" << std::endl;
  BundleAdjustmentByG2O ba2;
  ba2.testTcwOnlyPose();
  return 0;
}