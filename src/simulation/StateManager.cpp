#include "simulation/StateManager.h"

StateManager::StateManager(ros::NodeHandle nh, ParamsManager* params) {
  params_ = params;

  // Init random seed
  std::srand(time(nullptr));

  // Init state
  // TODO: give possibility to chose the init pose
  state_.setZero();

  // Set gps noise
  noise_gps_ = params_->gps_noise;

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

void StateManager::propagate(Eigen::Matrix<double, 3, 1> acceleration) {
  // Set state acceleration
  state_.tail(3) = acceleration;

  // Propagate the state
  state_ = F_ * state_;

  // Apply sensors noise
  apply_noise();
}

void StateManager::apply_noise() {
  Eigen::Vector3d rands;

  rands(0, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(1, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(2, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;

  // TODO: inline multiplication (if possible)
  Eigen::Vector3d noise;
  noise(0) = noise_gps_(0) * rands(0);
  noise(1) = noise_gps_(1) * rands(1);
  noise(2) = noise_gps_(2) * rands(2);

  state_gps_ = state_.head(3) + noise;
}

Eigen::Matrix<double, 3, 1> StateManager::get_gt() { return state_.head(3); }
Eigen::Matrix<double, 3, 1> StateManager::get_gps() { return state_gps_; }
