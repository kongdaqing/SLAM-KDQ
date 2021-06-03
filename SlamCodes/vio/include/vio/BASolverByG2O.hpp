//
// Created by kdq on 2021/5/29.
//
#pragma once
#include <iostream>
#include <random>
#include "Eigen/StdVector"
#include "opencv2/core/eigen.hpp"
#include "FeatureManager.hpp"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <sophus/se3.hpp>
using namespace vio;


 class VertexSBAPointXYZ : public g2o::BaseVertex<3, Eigen::Vector3d>
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexSBAPointXYZ(): g2o::BaseVertex<3, Eigen::Vector3d>() {};
  virtual bool read(std::istream& is) {
    Eigen::Vector3d lv;
    for (int i=0; i<3; i++)
      is >> _estimate[i];
    return true;
  };
  virtual bool write(std::ostream& os) const {
    Eigen::Vector3d lv=estimate();
    for (int i=0; i<3; i++){
      os << lv[i] << " ";
    }
    return os.good();
  };

  virtual void setToOriginImpl() {
    _estimate.fill(0.);
  }

  virtual void oplusImpl(const double* update)
  {
    Eigen::Map<const Eigen::Vector3d> v(update);
    _estimate += v;
  }
};

 // end namespace
/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  virtual void setToOriginImpl() override {
    _estimate = Sophus::SE3d();
  }

  /// left multiplication on SE3
  virtual void oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  virtual bool read(std::istream &in) override {}

  virtual bool write(std::ostream &out) const override {}
};

class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}

  virtual void computeError() override {
    const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
  }

  virtual void linearizeOplus() override {
    const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_cam = T * _pos3d;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double cx = _K(0, 2);
    double cy = _K(1, 2);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Z2 = Z * Z;
    _jacobianOplusXi
      << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
      0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
  }

  virtual bool read(std::istream &in) override {}

  virtual bool write(std::ostream &out) const override {}

 private:
  Eigen::Vector3d _pos3d;
  Eigen::Matrix3d _K;
};


class BASolverByG2O {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
  typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
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

  BASolverByG2O(double focalLength = 1.0,Eigen::Vector2d principlePoint = Eigen::Vector2d::Zero()) {
    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;  // pose is 6, landmark is 3
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    optimizer_.setAlgorithm(solver);   // 设置求解器
    optimizer_.setVerbose(true);       // 打开调试输出
    camera_ = new g2o::CameraParameters(focalLength,principlePoint,0);
    camera_->setId(0);
    optimizer_.addParameter(camera_);
  }

  void test() {
    g2o::VertexSE3Expmap *poseVertex = new g2o::VertexSE3Expmap();
    vertexId_ = 0;
    poseVertex->setId(vertexId_);
    Eigen::Vector3d t;
    t.setZero();
    Eigen::Matrix3d R;
    R.setIdentity();
    poseVertex->setEstimate(g2o::SE3Quat(R,t));
    optimizer_.addVertex(poseVertex);
    vertexId_++;

    std::vector<Eigen::Vector2d> meas;
    std::vector<Eigen::Vector3d> pt3ds;
    std::random_device sd;
    std::mt19937_64 genorator(sd());
    std::uniform_real_distribution<float> noise(-0.1,0.1);
    for (size_t i = 1; i < 11; i++) {
      Eigen::Vector3d point3d(sin(i),cos(i),1);
      Eigen::Vector3d meas3d = (R * point3d + t)  + 0.2 * Eigen::Vector3d(noise(genorator),noise(genorator),noise(genorator));
      pt3ds.push_back(point3d);
      meas.push_back(meas3d.head(2));
//      g2o::VertexPointXYZ *pt = new g2o::VertexPointXYZ();
//      pt->setId(i);
//      pt->setEstimate(point3d + Eigen::Vector3d(noise(genorator),noise(genorator),noise(genorator)));
//      optimizer_.addVertex(pt);
    }
    int featId = 1;
    for (auto m : meas) {
      g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
      edge->setVertex(0,dynamic_cast<g2o::VertexSE3Expmap*>(optimizer_.vertex(0)));
      //edge->setVertex(1,dynamic_cast<g2o::VertexPointXYZ*>(optimizer_.vertex(featId)));
      edge->setMeasurement(m);
      edge->setInformation(Eigen::Matrix2d::Identity());
      edge->setParameterId(0,0);
      edge->setRobustKernel(new g2o::RobustKernelHuber());
      optimizer_.addEdge(edge);
      edges_.push_back(edge);
      featId++;
    }
    std::cout<<"开始优化"<< std::endl;
    optimizer_.setVerbose(true);
    optimizer_.initializeOptimization();
    optimizer_.optimize(10);
    std::cout<<"优化完毕"<<std::endl;

  }



  void addVertexAndEdge(std::vector<FramePtr>& slidewindow,FeatureManager& fsm_) {
    if (slidewindow.size() < 2 || fsm_.getFeatureSize() < 3) {
      printf("[G2O-AddVertex]:Failure!SlideWindow size %ld and localMap size %d are not enough!\n",slidewindow.size(),fsm_.getFeatureSize());
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
      if (vertexId_ == 1) {
        vertexId_++;
        continue;
      }

      Eigen::Matrix3d eRwc;
      Eigen::Vector3d etwc;
      cv::Mat Rwc,twc;
      p->getPoseInWorld(Rwc,twc);
      cv::cv2eigen(Rwc,eRwc);
      cv::cv2eigen(twc,etwc);
      std::cout << "id = " << vertexId_ <<  " eRwc = \n" << eRwc << " \n eTwc = \n" << etwc.transpose() << std::endl;
      poseVertex->setEstimate(g2o::SE3Quat(eRwc,etwc));
      optimizer_.addVertex(poseVertex);
      poseId_[p] = vertexId_;
      vertexId_++;
    }
    VecVector3d pt3dAlign;
    std::map<uint64_t,Feature>& features = fsm_.getFeatureMap();
    for (std::map<uint64_t,Feature>::const_iterator it = features.begin();it != features.end();it++) {
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
      pt3dAlign.push_back(Eigen::Vector3d(pts3D.x,pts3D.y,pts3D.z));
      std::cout << "id = " << vertexId_ << " pts3d = " << pts3D << std::endl;
      pointVertex->setEstimate(pt3dAlign.back());
      optimizer_.addVertex(pointVertex);
      featId_[it->first] = vertexId_;
      vertexId_++;
    }
    for (auto p : poseId_) {
      FramePtr frame = p.first;
      int poseVertexId = p.second;
      VecVector2d pt2dAlign;
      for(auto f : featId_) {
        uint64_t featId = f.first;
        int featVertexId = f.second;
        if (!features.count(featId)) {
          continue;
        }
        PixelCoordinate uv;
        if (features[featId].getPixelInFrame(frame,uv)) {
          g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
          edge->setVertex(0,dynamic_cast<g2o::VertexSE3Expmap*>(optimizer_.vertex(poseVertexId)));
          edge->setVertex(1,dynamic_cast<g2o::VertexPointXYZ*>(optimizer_.vertex(featVertexId)));
          std::cout << "poseId = " << poseVertexId << " vs featId = " << featVertexId  << " vs meas = " << uv.second << std::endl;
          pt2dAlign.push_back(Eigen::Vector2d(uv.second.x,uv.second.y));
          edge->setMeasurement(pt2dAlign.back());
          edge->setInformation(Eigen::Matrix2d::Identity());
          edge->setParameterId(0,0);
          edge->setRobustKernel(new g2o::RobustKernelHuber());
          optimizer_.addEdge(edge);
          edges_.push_back(edge);
        }
      }
    }
    std::cout<<"开始优化"<< std::endl;
    optimizer_.setVerbose(true);
    optimizer_.initializeOptimization();
    optimizer_.optimize(10);
    std::cout<<"优化完毕"<<std::endl;
  }

  void getPoseAndFeatures(std::vector<FramePtr>& slidewindow,FeatureManager& fsm_) {
    for (auto p : slidewindow) {
      if (poseId_.count(p)) {
        g2o::VertexSE3Expmap * poseVertex = dynamic_cast<g2o::VertexSE3Expmap*> (optimizer_.vertex(poseId_[p]));
        Eigen::Isometry3d pose = poseVertex->estimate();
        cv::Mat Twc,Rwc,WtC;
        cv::eigen2cv(pose.matrix(),Twc);
        std::cout << "Twc = \n" << Twc << std::endl;
//        Twc.rowRange(0,3).colRange(0,3).copyTo(Rwc);
//        Twc.col(3).rowRange(0,3).copyTo(WtC);
//        p->setPoseInWorld(Rwc,WtC);
      }
    }
    std::map<uint64_t,Feature>& localMap = fsm_.getFeatureMap();
    for(auto iter = featId_.begin(); iter != featId_.end();iter++) {
      uint64_t id = iter->first;
      if (localMap.count(id)) {
        VertexSBAPointXYZ *pointVertex = dynamic_cast<VertexSBAPointXYZ *> (optimizer_.vertex(iter->second));
        Eigen::Vector3d pts3D = pointVertex->estimate();
        std::cout << "pts2d = \n" << pts3D << std::endl;
        //f.second.setPtsInWorld(cv::Point3f(pts3D.x(),pts3D.y(),pts3D.z()));
      }
    }
  }


 private:
  g2o::SparseOptimizer optimizer_;
  std::vector<g2o::EdgeProjectXYZ2UV*> edges_;
  g2o::CameraParameters *camera_;
  int vertexId_ = 0;
  std::map<FramePtr,int> poseId_;
  std::map<uint64_t,int> featId_;
};
