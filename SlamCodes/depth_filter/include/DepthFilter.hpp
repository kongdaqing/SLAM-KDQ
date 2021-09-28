// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SVO_DEPTH_FILTER_H_
#define SVO_DEPTH_FILTER_H_

#include <iostream>
#include <queue>
#include <map>
#include <list>
#include <thread>
#include <condition_variable>
#include <Eigen/Dense>
#include "Camera.hpp"

namespace DepthFilter {

class Corner;
typedef std::map<int,Corner> Corners;

/// Corner in images includes pixel coordinate and bearing vector etc
class Corner {
 public:
  Corner(){};
  Corner(int id,double d,Eigen::Vector3d f): 
    id_(id),
    d_(d),
    f_(f) {
  }
  Corner(const Corner& c) {
    d_ = c.d_;
    f_ = c.f_;
    id_ = c.id_;
  }
  ~Corner() {};
  double d_;
  Eigen::Vector3d f_;  //!< Unit-bearing vector of the feature.
  int id_;
};
/// Frame includes pose and features
class Frame {
 public:
  Frame(Eigen::Isometry3d T_f_w): T_f_w_(T_f_w) {};

  Corner* getCorner(int id) {
    if (corners_.count(id)) {
      return &corners_[id];
    } else {
      return nullptr;
    }
  } 
  Corners& getCorners() {
    return corners_;
  }
  void insertCorner(Corner& c) {
    if (!corners_.count(c.id_)) {
      corners_[c.id_] = c;
    }
  }
  Eigen::Isometry3d T_f_w_;
 private:
  Corners corners_;//这个会因为主函数析构而析构
};

/// A seed is a probabilistic depth estimate for a single pixel.
struct Seed
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Corner c;                //!< Corner in the keyframe for which the depth should be computed.
  float a;                     //!< a of Beta distribution: When high, probability of inlier is large.
  float b;                     //!< b of Beta distribution: When high, probability of outlier is large.
  float mu;                    //!< Mean of normal distribution.
  float z_range;               //!< Max range of the possible depth.
  float sigma2;                //!< Variance of normal distribution.
  Eigen::Isometry3d T_ref_w_;  //!< Translation from world to reference camera frame
  int lost_count;
  Seed(Corner& cor,Eigen::Isometry3d T_ref_w,float depth_mean, float depth_min);
};

/// Depth filter implements the Bayesian Update proposed in:
/// "Video-based, Real-Time Multi View Stereo" by G. Vogiatzis and C. Hernández.
/// In Image and Vision Computing, 29(7):434-441, 2011.
///
/// The class uses a callback mechanism such that it can be used also by other
/// algorithms than nslam and for simplified testing.
class DepthFilter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Frame> FramePtr;
  typedef std::unique_lock<std::mutex> lock_t;
  typedef std::function<void ( Eigen::Vector3d, double )> callback_t;

  /// Depth-filter config parameters
  struct Options
  {
    bool check_ftr_angle;                       //!< gradient features are only updated if the epipolar line is orthogonal to the gradient.
    bool epi_search_1d;                         //!< restrict Gauss Newton in the epipolar search to the epipolar line.
    bool verbose;                               //!< display output.
    bool use_photometric_disparity_error;       //!< use photometric disparity error instead of 1px error in tau computation.
    int max_lost_fs;                              //!< maximum number of keyframes for which we maintain seeds.
    double sigma_i_sq;                          //!< image noise.
    double seed_convergence_sigma2_thresh;      //!< threshold on depth uncertainty for convergence.
    Options()
    : check_ftr_angle(false),
      epi_search_1d(false),
      verbose(false),
      use_photometric_disparity_error(false),
      max_lost_fs(5),
      sigma_i_sq(5e-4),
      seed_convergence_sigma2_thresh(200.0)
    {}
  } options_;

  DepthFilter(std::string camConfigFile,callback_t seed_converged_cb,int width,int height);

  virtual ~DepthFilter();

  /// Start this thread when seed updating should be in a parallel thread.
  void startThread();

  /// Stop the parallel thread that is running.
  void stopThread();

  /// Add frame to the queue to be processed.
  void addFrame(FramePtr frame);

  /// Add new keyframe to the queue
  void addKeyframe(FramePtr frame, double depth_mean, double depth_min);

  /// If the map is reset, call this function such that we don't have pointers
  /// to old frames.
  void reset();

  /// Return a reference to the seeds. This is NOT THREAD SAFE!
  std::list<Seed>& getSeeds() { return seeds_; }

  /// Bayes update of the seed, x is the measurement, tau2 the measurement uncertainty
  static void updateSeed(
      const float x,
      const float tau2,
      Seed* seed);

  /// Compute the uncertainty of the measurement.
  //KDQ： 特征点在参考帧因为像素误差可能导致的和真值的误差
  static double computeTau(
      const Eigen::Isometry3d& T_ref_cur,
      const Eigen::Vector3d& f,
      const double z,
      const double px_error_angle);

protected:
  CameraPtr camPtr_;
  callback_t seed_converged_cb_;
  std::list<Seed> seeds_;
  std::map<int,bool> seedIds_;
  std::mutex seeds_mut_;
  bool seeds_updating_halt_;            //!< Set this value to true when seeds updating should be interrupted.
  std::thread* thread_;
  std::queue<FramePtr> frame_queue_;
  std::mutex frame_queue_mut_;
  std::condition_variable frame_queue_cond_;
  FramePtr new_keyframe_;               //!< Next keyframe to extract new seeds.
  bool new_keyframe_set_;               //!< Do we have a new keyframe to process?.
  double new_keyframe_min_depth_;       //!< Minimum depth in the new keyframe. Used for range in new seeds.
  double new_keyframe_mean_depth_;      //!< Maximum depth in the new keyframe. Used for range in new seeds.
  const int W;
  const int H;
  /// Initialize new seeds from a frame.
  void initializeSeeds(FramePtr frame);

  /// Update all seeds with a new measurement frame.
  virtual void updateSeeds(FramePtr frame);

  /// When a new keyframe arrives, the frame queue should be cleared.
  void clearFrameQueue();

  /// A thread that is continuously updating the seeds.
  void updateSeedsLoop();

  bool depthFromTriangulation(
    const Eigen::Isometry3d& T_search_ref,
    const Eigen::Vector3d& f_ref,
    const Eigen::Vector3d& f_cur,
    double& depth);

  inline bool isInFrame(Eigen::Vector2i uv) {  
    return uv.x() > 1 && uv.x() < W && uv.y() > 1 && uv.y() < H;
  }

  inline Eigen::Vector2d f2c(Eigen::Vector3d f) {
    double u = camPtr_->fx() * f.x()/f.z() + camPtr_->cx();
    double v = camPtr_->fy() * f.y()/f.z() + camPtr_->cy();
    return Eigen::Vector2d(u,v);
  }
  
};

} // namespace svo

#endif // SVO_DEPTH_FILTER_H_
