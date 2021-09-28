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

#include "DepthFilter.hpp"
#include <algorithm>
#include <boost/math/distributions/normal.hpp>


namespace DepthFilter {

Seed::Seed(Corner& cor,Eigen::Isometry3d T_ref_w,float depth_mean, float depth_min) :
    T_ref_w_(T_ref_w),
    c(cor),
    a(10),
    b(10),
    mu(1.0/depth_mean),
    z_range(1.0/depth_min),
    sigma2(z_range*z_range/36),
    lost_count(0) {
}

DepthFilter::DepthFilter(std::string cameraConfigFile,callback_t seed_converged_cb,int width,int height) :
    seed_converged_cb_(seed_converged_cb),
    W(width),
    H(height),
    seeds_updating_halt_(false),
    thread_(NULL),
    new_keyframe_set_(false),
    new_keyframe_min_depth_(0.0),
    new_keyframe_mean_depth_(0.0) {
  if(cameraConfigFile.empty()) {
    camPtr_ = nullptr;
  } else {
    camPtr_.reset(new Camera(cameraConfigFile));
  }
}

DepthFilter::~DepthFilter() {
  stopThread();
  printf("DepthFilter destructed.");
}

void DepthFilter::startThread()
{
  thread_ = new std::thread(&DepthFilter::updateSeedsLoop, this);
}

void DepthFilter::stopThread()
{
  printf("DepthFilter stop thread invoked.");
  if(thread_ != NULL)
  {
    //KDQ: 不知道如何打断
    // printf("DepthFilter interrupt and join thread... ");
    // seeds_updating_halt_ = true;
    // thread_->interrupt();
    // thread_->join();
    // thread_ = NULL;
  }
}

void DepthFilter::addFrame(FramePtr frame)
{
  if(thread_ != NULL) {
    {
      lock_t lock(frame_queue_mut_);
      if(frame_queue_.size() > 2)
        frame_queue_.pop();
      frame_queue_.push(frame);
    }
    seeds_updating_halt_ = false;
    frame_queue_cond_.notify_one();
  } else {
    updateSeeds(frame);
  }
}

void DepthFilter::addKeyframe(FramePtr frame, double depth_mean, double depth_min)
{
  new_keyframe_min_depth_ = depth_min;
  new_keyframe_mean_depth_ = depth_mean;
  if(thread_ != NULL) {
    new_keyframe_ = frame;
    new_keyframe_set_ = true;
    seeds_updating_halt_ = true;
    frame_queue_cond_.notify_one();
  } else { 
    initializeSeeds(frame);
  }
}

void DepthFilter::initializeSeeds(FramePtr frame)
{
  // initialize a seed for every new feature
  seeds_updating_halt_ = true;
  lock_t lock(seeds_mut_); // by locking the updateSeeds function stops
  Corners& feats = frame->getCorners();
  for (auto it = feats.begin();it != feats.end();it++) {
    if (!seedIds_.count(it->second.id_)) {
      seeds_.push_back(Seed(*(&(it->second)),frame->T_f_w_, new_keyframe_mean_depth_, new_keyframe_min_depth_));
      seedIds_[it->second.id_] = true; 
    }
  }
  seeds_updating_halt_ = false;
}

void DepthFilter::reset()
{
  seeds_updating_halt_ = true;
  {
    lock_t lock(seeds_mut_);
    seeds_.clear();
    seedIds_.clear();
  }
  lock_t lock();
  while(!frame_queue_.empty())
    frame_queue_.pop();
  seeds_updating_halt_ = false;

  if(options_.verbose)
    printf("DepthFilter: RESET.");
}

void DepthFilter::updateSeedsLoop()
{
  while(1) {
    FramePtr frame;
    {
      lock_t lock(frame_queue_mut_);
      while(frame_queue_.empty() && new_keyframe_set_ == false)
        frame_queue_cond_.wait(lock);
      if(new_keyframe_set_) {
        new_keyframe_set_ = false;
        seeds_updating_halt_ = false;
        clearFrameQueue();
        frame = new_keyframe_;
      } else {
        frame = frame_queue_.front();
        frame_queue_.pop();
      }
    }
    updateSeeds(frame);
    initializeSeeds(frame);
  }
}

bool DepthFilter::depthFromTriangulation(
    const Eigen::Isometry3d& T_search_ref,
    const Eigen::Vector3d& f_ref,
    const Eigen::Vector3d& f_cur,
    double& depth) {
  Eigen::Matrix<double,3,2> A; A << T_search_ref.rotation() * f_ref, f_cur;
  const Eigen::Matrix2d AtA = A.transpose()*A;
  if(AtA.determinant() < 0.000001)
    return false;
  const Eigen::Vector2d depth2 = - AtA.inverse()*A.transpose()*T_search_ref.translation();
  depth = fabs(depth2[0]);
  return true;
}


void DepthFilter::updateSeeds(FramePtr frame)
{
  // update only a limited number of seeds, because we don't have time to do it
  // for all the seeds in every frame!
  size_t n_updates = 0, n_failed_matches = 0, n_seeds = seeds_.size();
  lock_t lock(seeds_mut_);
  std::list<Seed>::iterator it = seeds_.begin();

  const double focal_length = camPtr_->fx();
  double px_noise = 1.0;
  double px_error_angle = atan(px_noise/(2.0*focal_length))*2.0; // law of chord (sehnensatz)

  while( it != seeds_.end())
  {
    // set this value true when seeds updating should be interrupted
    if(seeds_updating_halt_)
      return;
    // check if seed is not already too old
    // KDQ: 对于rovio来说这个是否合理，是不是判断看不到这个特征点再剔除
    if(it->lost_count > options_.max_lost_fs) {
      if (seedIds_.count(it->c.id_)) {
        seedIds_.erase(it->c.id_);
      }
      it = seeds_.erase(it);
      continue;
    }
    Corner * curFtr = frame->getCorner(it->c.id_);
    if (curFtr == nullptr) {
      it->b++; // increase outlier probability when no match was found
      it->lost_count++;
      ++it;
      ++n_failed_matches;
      continue;
    }
    // check if point is visible in the current image
    Eigen::Isometry3d T_ref_cur = it->T_ref_w_ * frame->T_f_w_.inverse();
    const Eigen::Vector3d xyz_f(T_ref_cur.inverse()*(1.0/it->mu * it->c.f_) );
    if(xyz_f.z() < 0.0)  {
      ++it; // behind the camera
      continue;
    }
    if(!isInFrame(f2c(xyz_f).cast<int>())) {
      ++it; // point does not project in image
      continue;
    }

    // we are using inverse depth coordinates
    float z_inv_min = it->mu + sqrt(it->sigma2);
    float z_inv_max = std::max(it->mu - sqrt(it->sigma2), 0.00000001f);
    double z;
    if(!depthFromTriangulation(T_ref_cur,it->c.f_,curFtr->f_, z)) {
      it->b++; // increase outlier probability when no match was found
      ++it;
      ++n_failed_matches;
      continue;
    }
    // compute tau
    double tau = computeTau(T_ref_cur, it->c.f_, z, px_error_angle);
    double tau_inverse = 0.5 * (1.0/std::max(0.0000001, z-tau) - 1.0/(z+tau));

    // update the estimate
    updateSeed(1./z, tau_inverse*tau_inverse, &*it);
    ++n_updates;

     // if the seed has converged, we initialize a new candidate point and remove the seed
    if(sqrt(it->sigma2) < it->z_range/options_.seed_convergence_sigma2_thresh) {
      //assert(it->c.point == NULL); // TODO this should not happen anymore
      Eigen::Vector3d xyz_world(it->T_ref_w_.inverse() * (it->c.f_ * (1.0/it->mu)));
      {
        seed_converged_cb_(xyz_world, it->sigma2); // put in candidate list
      }
      if (seedIds_.count(it->c.id_)) {
        seedIds_.erase(it->c.id_);
      }
      it = seeds_.erase(it);
    }  else if (isnan(z_inv_min)) {
      printf("z_min is NaN");
      if (seedIds_.count(it->c.id_)) {
        seedIds_.erase(it->c.id_);
      }
      it = seeds_.erase(it);
    } else {
      ++it;
    }
  }
}

void DepthFilter::clearFrameQueue()
{
  while(!frame_queue_.empty())
    frame_queue_.pop();
}

void DepthFilter::updateSeed(const float x, const float tau2, Seed* seed)
{
  float norm_scale = sqrt(seed->sigma2 + tau2);
  if(std::isnan(norm_scale))
    return;
  boost::math::normal_distribution<float> nd(seed->mu, norm_scale);
  float s2 = 1./(1./seed->sigma2 + 1./tau2);
  float m = s2*(seed->mu/seed->sigma2 + x/tau2);
  float C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);
  float C2 = seed->b/(seed->a+seed->b) * 1./seed->z_range;
  float normalization_constant = C1 + C2;
  C1 /= normalization_constant;
  C2 /= normalization_constant;
  float f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
  float e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
          + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));

  // update parameters
  float mu_new = C1*m+C2*seed->mu;
  seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
  seed->mu = mu_new;
  seed->a = (e-f)/(f-e/f);
  seed->b = seed->a*(1.0f-f)/f;
}

double DepthFilter::computeTau(
      const Eigen::Isometry3d& T_ref_cur,
      const Eigen::Vector3d& f,
      const double z,
      const double px_error_angle)
{
  Eigen::Vector3d t(T_ref_cur.translation());
  Eigen::Vector3d a = f*z-t;
  double t_norm = t.norm();
  double a_norm = a.norm();
  double alpha = acos(f.dot(t)/t_norm); // dot product
  double beta = acos(a.dot(-t)/(t_norm*a_norm)); // dot product
  double beta_plus = beta + px_error_angle;
  double gamma_plus = M_PI - alpha-beta_plus; // triangle angles sum to PI
  double z_plus = t_norm*sin(beta_plus)/sin(gamma_plus); // law of sines
  return (z_plus - z); // tau
}

} // namespace svo
