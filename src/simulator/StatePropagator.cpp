#include "simulator/StatePropagator.h"

StatePropagator::StatePropagator(ParamsManager* params) {
  params_ = params;

  // Set random seed
  std::srand(time(nullptr));

  // Init the state vector
  state_.setZero();
  state_.head(3) << params_->init_pose;

  // Set the ground truth time
  dt_ = 1 / params_->frequency_gt;
  double dt2 = std::pow(dt_, 2) * 0.5;

  // Init of the State-transistion matrix
  F_.block<1, 6>(0, 0) << 1, 0, 0, dt_, 0, 0;
  F_.block<1, 6>(1, 0) << 0, 1, 0, 0, dt_, 0;
  F_.block<1, 6>(2, 0) << 0, 0, 1, 0, 0, dt_;
  F_.block<1, 6>(3, 0) << 0, 0, 0, 1, 0, 0;
  F_.block<1, 6>(4, 0) << 0, 0, 0, 0, 1, 0;
  F_.block<1, 6>(5, 0) << 0, 0, 0, 0, 0, 1;

  // Init of the Control-input matrix
  B_.block<1, 3>(0, 0) << dt2, 0, 0;
  B_.block<1, 3>(1, 0) << 0, dt2, 0;
  B_.block<1, 3>(2, 0) << 0, 0, dt2;
  B_.block<1, 3>(3, 0) << dt_, 0, 0;
  B_.block<1, 3>(4, 0) << 0, dt_, 0;
  B_.block<1, 3>(5, 0) << 0, 0, dt_;
}

void StatePropagator::propagate(const Eigen::Vector3d& acceleration) {
  // Propagate the state
  state_ = F_ * state_ + B_ * acceleration;
}

Eigen::Vector3d StatePropagator::generate_gps() {
  Eigen::Vector3d state_gps;
  Eigen::Vector3d rands;
  Eigen::Vector3d noise;

  rands(0, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(1, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(2, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;

  // Compute sensor noise
  noise = params_->noise_gps.cwiseProduct(rands);

  // Generate GPS measurement w/ noise
  state_gps = state_.head(3) + noise;

  return state_gps;
}

Eigen::Vector3d StatePropagator::generate_radar() {
  // Generate random nums
  Eigen::Vector3d state_radar;
  Eigen::Vector3d rands;
  Eigen::Vector3d noise;
  rands(0, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(1, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(2, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;

  // Compute sensor noise
  noise = params_->noise_radar.cwiseProduct(rands);

  // From global to local coordinates
  double x = state_(0, 0) - params_->init_pose_radar(0, 0);
  double y = state_(1, 0) - params_->init_pose_radar(1, 0);
  double z = state_(2, 0) - params_->init_pose_radar(2, 0);

  // From local cartesian coordinates to local spherical coordinates
  state_radar(0, 0) =
      std::pow(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2), 0.5);
  state_radar(1, 0) = std::acos(z / state_radar(0, 0));
  state_radar(2, 0) = std::atan(y / (x + 0.0001));

  // Add mesurement noise
  state_radar += noise;

  return state_radar;
}

Eigen::Vector3d StatePropagator::get_gt() { return state_.head(3); }