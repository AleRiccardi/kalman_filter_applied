#include "simulator/StatePropagator.h"

StatePropagator::StatePropagator(ParamsManager* params) {
  params_ = params;

  // Init random seed
  std::srand(time(nullptr));

  // Initial state
  state_.setZero();
  state_.head(3) << params_->init_pose;

  // Initial radar state
  state_radar_.setZero();
  // Initial radar pose
  location_radar_ << params_->init_pose_radar;

  // Set gps noise
  noise_gps_ = params_->noise_gps;
  noise_radar_ = params_->noise_radar;

  // Init propagation matrix
  double dt2 = std::pow(dt, 2) * 0.5;
  F_.block<1, 9>(0, 0) << 1, 0, 0, dt, 0, 0, dt2, 0, 0;
  F_.block<1, 9>(1, 0) << 0, 1, 0, 0, dt, 0, 0, dt2, 0;
  F_.block<1, 9>(2, 0) << 0, 0, 1, 0, 0, dt, 0, 0, dt2;
  F_.block<1, 9>(3, 0) << 0, 0, 0, 1, 0, 0, dt, 0, 0;
  F_.block<1, 9>(4, 0) << 0, 0, 0, 0, 1, 0, 0, dt, 0;
  F_.block<1, 9>(5, 0) << 0, 0, 0, 0, 0, 1, 0, 0, dt;
  F_.block<1, 9>(6, 0) << 0, 0, 0, 0, 0, 0, 1, 0, 0;
  F_.block<1, 9>(7, 0) << 0, 0, 0, 0, 0, 0, 0, 1, 0;
  F_.block<1, 9>(8, 0) << 0, 0, 0, 0, 0, 0, 0, 0, 1;
}

void StatePropagator::propagate(const Eigen::Vector3d& acceleration) {
  // Set state acceleration
  state_.tail(3) = acceleration;

  // Propagate the state
  state_ = F_ * state_;

  // To be called by the SimManager
  generate_gps();
  generate_radar();
}

void StatePropagator::generate_gps() {
  Eigen::Vector3d rands;
  rands(0, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(1, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(2, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;

  // Compute sensor noise
  Eigen::Vector3d noise = noise_gps_.cwiseProduct(rands);

  state_gps_ = state_.head(3) + noise;
}

void StatePropagator::generate_radar() {
  // Generate random nums
  Eigen::Vector3d rands;
  rands(0, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(1, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(2, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;

  // Compute sensor noise
  Eigen::Vector3d noise = noise_radar_.cwiseProduct(rands);

  // From global to local coordinates
  double x = state_(0, 0) - location_radar_(0, 0);
  double y = state_(1, 0) - location_radar_(1, 0);
  double z = state_(2, 0) - location_radar_(2, 0);

  // From local cartesian coordinates to local polar coordinates
  state_radar_(0, 0) =
      std::pow(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2), 0.5);
  state_radar_(1, 0) = std::acos(z / state_radar_(0, 0));
  state_radar_(2, 0) = std::atan(y / (x + 0.0001));

  // Add mesurement noise
  state_radar_ += noise;
}

Eigen::Vector3d StatePropagator::get_gt() { return state_.head(3); }
Eigen::Vector3d StatePropagator::get_gps() { return state_gps_; }
Eigen::Vector3d StatePropagator::get_radar() { return state_radar_; }
