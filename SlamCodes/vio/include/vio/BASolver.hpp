#pragma once
#include <iostream>
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "Frame.hpp"
#include "FeatureManager.hpp"
#include "Camera.hpp"
#include <opencv2/core/eigen.hpp>
using namespace vio;
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
G2O_USE_OPTIMIZATION_LIBRARY(csparse)
class BAG2O {
 public:
  BAG2O(std::string solverName = "lm_fix6_3_csparse", std::string robustCoreName = "Huber", bool structureOnly = false) {
    std::string solverType = solverName;
    robustType_ = robustCoreName;
    structureOnly_ = structureOnly;
    g2o::OptimizationAlgorithmProperty solverProperty;
    optimizer_.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct(solverType, solverProperty));
  }

  bool windowFrameOptimize(std::vector<FramePtr>& slidewindow,FeatureManager& fsm,double pixelErr,
                           double focalLength = 1.,Eigen::Vector2d origin = Eigen::Vector2d::Zero()) {
    if (slidewindow.size() < 2) {
      printf("[G2O-AddVertex]:Failure!SlideWindow size %ld is not enough!\n",slidewindow.size());
      return false;
    }

    g2o::CameraParameters * cam_params = new g2o::CameraParameters (focalLength,origin,0);
    double informationGain = 1.0 / (pixelErr * pixelErr);
    cam_params->setId(0);
    if (!optimizer_.addParameter(cam_params)) {
      assert(false);
    }    //add pose vertex
    int vertexId = 0;
    for (auto s : slidewindow) {
      g2o::VertexSE3Expmap * v_cam = new g2o::VertexSE3Expmap();
      v_cam->setId(vertexId);
      if (vertexId == 0) {
        v_cam->setFixed(true);
      }
      //add estimate
      Eigen::Matrix3d eRcw;
      Eigen::Vector3d etcw;
      cv::Mat Rcw,tcw;
      s->getInversePose(Rcw,tcw);
      if (Rcw.empty() || tcw.empty()) {
        std::cerr << "[BA]:Failure!This frame pose is not set!" << std::endl;
        return false;
      }
      cv::cv2eigen(Rcw,eRcw);
      cv::cv2eigen(tcw,etcw);
      g2o::SE3Quat Tcw(eRcw,etcw);
      v_cam->setEstimate(Tcw);
      optimizer_.addVertex(v_cam);
      poseId_[s] = vertexId;
      vertexId++;
    }
    std::map<uint64_t,Feature>& features = fsm.getFeatureMap();
    for (std::map<uint64_t,Feature>::const_iterator it = features.begin();it != features.end();it++) {
      const Feature& fea = it->second;
      const uint64_t idx = it->first;
      if (!fea.isReadyForOptimize()) {
        continue;
      }

      cv::Point3f ft3d = fea.getPts3DInWorld();
      g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
      v_p->setId(vertexId);
      if (fea.getGoodCount() >= 3) {
        v_p->setFixed(true);
      }
      v_p->setEstimate(Eigen::Vector3d(ft3d.x,ft3d.y,ft3d.z));
      v_p->setMarginalized(true);
      optimizer_.addVertex(v_p);
      featId_[idx] = vertexId;
      vertexId++;
    }
    for (auto s : slidewindow) {
      if (!poseId_.count(s)) {
        continue;
      }
      int poseVId = poseId_[s];
      for (std::map<uint64_t,int>::const_iterator it = featId_.begin(); it != featId_.end(); it++) {
        uint64_t idx = it->first;
        int featVId = it->second;
        if (!features.count(idx)) {
          continue;
        }
        PixelCoordinate uv;
        if (features[idx].getPixelInFrame(s,uv)) {
          cv::Point2f z;
          if (origin.norm() < 1) {
            z = uv.second;
          } else {
            z = uv.first;
          }
          g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
          edge->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(featVId)));
          edge->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(poseVId)));
          edge->setMeasurement(Eigen::Vector2d(z.x,z.y));
          edge->setInformation(Eigen::Matrix2d::Identity() * informationGain);
          edge->setRobustKernel(new g2o::RobustKernelHuber());
          edge->setParameterId(0,0);
          optimizer_.addEdge(edge);
        }
      }
    }
    optimizer_.setVerbose(true);
    optimizer_.initializeOptimization();
    optimizer_.optimize(5);
    return true;
  }
  void updatePoseAndMap(std::vector<FramePtr>& slidewindow,FeatureManager& fsm) {
    for (auto f : slidewindow) {
      if (!poseId_.count(f)) {
        continue;
      }
      int poseVId = poseId_[f];
      g2o::VertexSE3Expmap* poseV = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer_.vertex(poseVId));
      Eigen::Matrix3d Rwc = poseV->estimate().rotation().toRotationMatrix().transpose();
      Eigen::Vector3d twc = -Rwc * poseV->estimate().translation();
      cv::Mat cRwc,ctwc;
      cv::eigen2cv(Rwc, cRwc);
      cv::eigen2cv(twc, ctwc);
      f->setPoseInWorld(cRwc,ctwc);
    }
    std::map<uint64_t,Feature>& features = fsm.getFeatureMap();
    for (auto c : featId_) {
      uint64_t cornerId = c.first;
      int featVId = c.second;
      if (!features.count(cornerId)) {
        continue;
      }
      g2o::VertexPointXYZ* pointV = dynamic_cast<g2o::VertexPointXYZ*> (optimizer_.vertex(featVId));
      Eigen::Vector3d pt3d = pointV->estimate();
      cv::Point3f pt3old = features[cornerId].getPts3DInWorld();
      std::cout << "Optimization: old feat = " << pt3old << " new feat = " << pt3d.transpose() << std::endl;
      assert(!isnan(pt3d.x()));
      features[cornerId].setPtsInWorld(cv::Point3f(pt3d.x(),pt3d.y(),pt3d.z()));
    }
  }


 private:
  g2o::SparseOptimizer optimizer_;
  std::string robustType_;
  bool structureOnly_;
  std::map<FramePtr,int> poseId_;
  std::map<uint64_t,int> featId_;
};