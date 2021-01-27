#ifndef KF_APPLIED_STATE_MANAGER_H
#define KF_APPLIED_STATE_MANAGER_H

#include <math.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "utils/ParamsManager.h"
#include "utils/types.h"

class StatePropagator {
 public:
  StatePropagator(ros::NodeHandle nh, ParamsManager* params);
  ~StatePropagator() = default;

  void propagate(Eigen::Vector3d acceleration);
  void generate_gps();
  void generate_radar();

  Eigen::Vector3d get_gt();
  Eigen::Vector3d get_gps();
  Eigen::Vector3d get_radar();

  double dt = 0.2;

 private:
  ParamsManager* params_;

  Eigen::Matrix<double, 9, 9> F_;
  Eigen::Vector3d noise_gps_;
  Eigen::Vector3d noise_radar_;

  int poses_count_ = 0;
  Eigen::Matrix<double, 9, 1> state_;
  Eigen::Vector3d state_gps_;
  Eigen::Vector3d state_radar_;

  Eigen::Vector3d location_radar_;
};

#endif  // KF_APPLIED_STATE_MANAGER_H