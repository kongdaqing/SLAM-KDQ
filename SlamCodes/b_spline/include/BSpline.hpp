//
// Created by kdq on 2021/6/16.
//
#pragma once
#include <iostream>
#include <datatable.h>
#include <bspline.h>
#include <bsplinebuilder.h>
#include <fstream>
#include <vector>
using namespace SPLINTER;
template <int S,int D,int K>
class BSplineX {
 public:
  static const int S_ = S;
  static const int D_ = D;
  static const int K_ = K;
  static_assert(K_ < 6);
  BSplineX(Eigen::Matrix<double,S_,1>& xVec,Eigen::Matrix<double,S_,D_>& yMatrix) {
    for (int i = 0; i < D_; i++) {
      DataTable samples;
      for (int j = 0; j < S_, j++) {
        samples.addSample(xVec(j,0),yMatrix(j,i));
      }
      BSpline * sp = new BSpline::Builder(samples).degree(K_).build();
      splineVec.push_back(sp);
    }
    xBegin_ = xVec(0,0);
    XEnd_ = xVec(S_-1,0);
  }

  template<int YDIM>
  double getEvalValue(double x) {
    static_assert(YDIM < D_);
    DenseVector t(1);
    t(0) = x;
    return splineVec[YDIM]->eval(x);
  }

  template<int YDIM>
  double getFirstDifference(double x) {
    if ( x < xBegin_ || x > xEnd_) {
      return 0;
    }
    static_assert(YDIM < D_);
    DenseVector t(1);
    t(0) = x;
    DenseMatrix v;
    v = splineVec[YDIM]->centralDifference(t);
    return v(0,0);
  }


 private:
  std::vector<BSpline*> splineVec;
  double xBegin_,xEnd_;
};



