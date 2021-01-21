#ifndef KF_APPLIED_STATE_MANAGER_H
#define KF_APPLIED_STATE_MANAGER_H

#include <math.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "utils/ParamsManager.h"

class StateManager {
 public:
  StateManager(ros::NodeHandle nh, ParamsManager* params);
  ~StateManager() = default;

  void propagate(Eigen::Matrix<double, 3, 1> acceleration);
  void generate_gps();
  void generate_radar();

  Eigen::Matrix<double, 3, 1> get_gt();
  Eigen::Matrix<double, 3, 1> get_gps();

  double dt = 0.2;

 private:
  ParamsManager* params_;

  Eigen::Matrix<double, 9, 9> F_;
  Eigen::Vector3d noise_gps_;
  Eigen::Vector3d noise_radar_;

  int poses_count_ = 0;
  Eigen::Matrix<double, 9, 1> state_;
  Eigen::Matrix<double, 3, 1> state_gps_;
  Eigen::Matrix<double, 3, 1> state_radar_;

  Eigen::Matrix<double, 3, 1> pose_radar_;
};

#endif  // KF_APPLIED_STATE_MANAGER_H