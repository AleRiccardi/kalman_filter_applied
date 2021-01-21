#include "simulation/StateManager.h"

StateManager::StateManager(ros::NodeHandle nh, ParamsManager* params) {
  params_ = params;

  // Init random seed
  std::srand(time(nullptr));

  // Initial state
  state_.setZero();
  state_.head(3) << params_->init_pose;

  // Initial radar state
  state_radar_.setZero();
  // Initial radar pose
  pose_radar_ << params_->init_pose_radar;

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

void StateManager::propagate(Eigen::Matrix<double, 3, 1> acceleration) {
  // Set state acceleration
  state_.tail(3) = acceleration;

  // Propagate the state
  state_ = F_ * state_;

  // To be called by the SimManager
  generate_gps();
  generate_radar();
}

void StateManager::generate_gps() {
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

void StateManager::generate_radar() {
  // Generate random nums
  Eigen::Vector3d rands;
  rands(0, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(1, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(2, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;

  // Compute sensor noise
  // TODO: inline multiplication (if possible)
  Eigen::Vector3d noise;
  noise(0) = noise_radar_(0) * rands(0);
  noise(1) = noise_radar_(1) * rands(1);
  noise(2) = noise_radar_(2) * rands(2);

  state_radar_(0, 0) =
      std::pow(std::pow(state_(0, 0) - pose_radar_(0, 0), 2) +
                   std::pow(state_(1, 0) - pose_radar_(1, 0), 2),
               0.5);
  state_radar_(1, 0) = std::atan((state_(1, 0) - pose_radar_(0, 0)) /
                                 (state_(0, 0) - pose_radar_(0, 0) + 0.0001));

  state_radar_ += noise;
}

Eigen::Matrix<double, 3, 1> StateManager::get_gt() { return state_.head(3); }
Eigen::Matrix<double, 3, 1> StateManager::get_gps() { return state_gps_; }