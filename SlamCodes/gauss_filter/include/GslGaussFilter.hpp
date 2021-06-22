//
// Created by kdq on 2021/6/22.
//
#pragma once
#include <vector>
#include "Eigen/Core"
#include <gsl/gsl_filter.h>
#include <gsl/gsl_vector.h>
#include "iostream"
class GslGaussFilter {
 public:
  /** \brief set gauss filter through gsl library
   *
   * @param windowSize -- gauss filter window size
   * @param alpha -- alpha = (windowSize - 1) / (2 * standard deviation)
   */
  GslGaussFilter(size_t windowSize,double alpha):K_(windowSize),alpha_(alpha) {
    f_ = gsl_filter_gaussian_alloc(K_);

  }

  std::vector<double> apply(std::vector<double>& rawData) {
    gsl_vector *x = gsl_vector_alloc(rawData.size());
    gsl_vector *y = gsl_vector_alloc(rawData.size());
    for (size_t i = 0; i < rawData.size(); i++) {
      gsl_vector_set(x, i, rawData[i]);
    }
    gsl_filter_gaussian(GSL_FILTER_END_PADVALUE, alpha_, 0, x, y, f_);
    std::vector<double> outData;
    for (size_t i = 0; i < rawData.size(); i++) {
      outData.push_back(gsl_vector_get(y, i));
    }
    gsl_vector_free(x);
    gsl_vector_free(y);
    return outData;
  }


  std::vector<Eigen::VectorXd> apply(std::vector<Eigen::VectorXd>& rawData) {
    if (rawData.empty()) {
      return rawData;
    }
    const size_t D = rawData.front().rows();
    const size_t E = rawData.size();
    std::vector<std::vector<double>> yy;
    for (size_t j = 0; j < D; j++) {
      std::vector<double> x, y;
      for (size_t i = 0; i < E; i++) {
        x.push_back(rawData[i](j));
      }
      y = apply(x);
      yy.push_back(y);
    }
    std::vector<Eigen::VectorXd> outData;
    Eigen::VectorXd out;
    out.resize(D);
    for (size_t i = 0; i < E; i++) {
      for (size_t j = 0; j < D; j++) {
        out(j) = yy[j][i];
      }
      outData.push_back(out);
    }
    return outData;
  }

  ~GslGaussFilter() {
    gsl_filter_gaussian_free(f_);
  }

 private:
  const size_t K_;
  const double alpha_;
  gsl_filter_gaussian_workspace *f_;
};
