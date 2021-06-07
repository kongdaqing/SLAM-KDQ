#include <iostream>
#include "BundleAdjustmentByG2O.hpp"
int main() {
  BundleAdjustmentByG2O ba;
  ba.testTcw();
  std::cout << "hello g2o" << std::endl;
  return 0;
}
