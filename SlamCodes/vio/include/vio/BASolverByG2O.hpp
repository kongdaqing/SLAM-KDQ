//
// Created by kdq on 2021/5/29.
//
#pragma once
#include "opencv2/core/eigen.hpp"
#include "FeatureManager.hpp"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
using namespace vio;


class BASolverByG2O {
 public:
  enum LinerSolver {
    CSparse,
    Cholmod,
    PCG
  };
  enum OptimizeAlgorithm {
    GaussNewton,
    LevenbergMarquardt,
    Dogleg,
  };

  BASolverByG2O(double focalLength,Eigen::Vector2d principlePoint) {
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
    g2o::BlockSolver_6_3* blockSolver = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    optimizer_.setAlgorithm(algorithm);
    camera_ = new g2o::CameraParameters(focalLength,principlePoint,0);
    camera_->setId(0);
    optimizer_.addParameter(camera_);
  }

  void addVertexAndEdge(const std::vector<FramePtr>& slidewindow,const FeatureManager& fsm_) {
    if (slidewindow.empty() || fsm_.getFeatureSize() < 3) {
      printf(("[G2O-AddVertex]:Failure!SlideWindow size %d and localMap size %d are not enough!\n",slidewindow.size(),fsm_.getFeatureSize());
      return;
    }
    //add pose vertex
    vertexId_ = 0;
    for (auto p : slidewindow) {
      g2o::VertexSE3Expmap *poseVertex = new g2o::VertexSE3Expmap();
      poseVertex->setId(vertexId_);
      if (vertexId_ == 0) {
        poseVertex->setFixed(true);
      }
      Eigen::Matrix3d eRwc;
      Eigen::Vector3d etwc;
      cv::Mat Rwc,twc;
      p->getPoseInWorld(Rwc,twc);
      cv::cv2eigen(Rwc,eRwc);
      cv::cv2eigen(twc,etwc);
      poseVertex->setEstimate(g2o::SE3Quat(eRwc,etwc));
      optimizer_.addVertex(poseVertex);
      poseId_[p] = vertexId_;
      vertexId_++;
    }
    const std::map<uint64,Feature>& features = fsm_.getFeatureMap();
    for (std::map<uint64,Feature>::const_iterator it = features.begin();it != features.end();it++) {
      const Feature& fea = it->second;
      if (!fea.isReadyForOptimize()) {
        continue;
      }
      g2o::VertexPointXYZ *pointVertex = new g2o::VertexPointXYZ();
      pointVertex->setId(vertexId_);
      if (fea.getTrackCount() > 5) {
        pointVertex->setFixed(true);
      }
      cv::Point3f pts3D = fea.getPts3DInWorld();
      pointVertex->setEstimate(Eigen::Vector3d(pts3D.x,pts3D.y,pts3D.z));
      optimizer_.addVertex(pointVertex);
      featId_[it->first] = vertexId_;
      vertexId_++;
    }
    std::vector<g2o::EdgeProjectXYZ2UV*> edges;
    for (auto p : slidewindow) {
      std::vector<cv::Point2f> matchedNormalizedUV;
      std::vector<cv::Point3f> matchedPts3D;
      std::vector<uint64_t> matchedIds;
      fsm_.featureMatching(cam_,frame,matchedIds,matchedNormalizedUV,matchedPts3D);
      if (matchedPts3D.size() < 5) {
        printf("[EstimatePose]: matched points is too few!\n");
        return false;
      }
    }
  }


 private:
  g2o::SparseOptimizer optimizer_;
  g2o::CameraParameters *camera_;
  int vertexId_ = 0;
  std::map<FramePtr,int> poseId_;
  std::map<uint64_t,int> featId_;
};
