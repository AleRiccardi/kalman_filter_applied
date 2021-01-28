#ifndef KFA_STATE_PROPAGATOR_H
#define KFA_STATE_PROPAGATOR_H

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
  StatePropagator(ParamsManager* params);
  ~StatePropagator() = default;

  void propagate(const Eigen::Vector3d& acceleration);
  Eigen::Vector3d get_gt();
  Eigen::Vector3d generate_gps();
  Eigen::Vector3d generate_radar();

 private:
  Eigen::Vector3d random_probs();

  ParamsManager* params_;

  double dt_;
  int poses_count_ = 0;

  // State vector
  Eigen::Matrix<double, 6, 1> state_;

  // State-transistion matrix
  Eigen::Matrix<double, 6, 6> F_;
  // Control-input matrix
  Eigen::Matrix<double, 6, 3> B_;
};

#endif  // KFA_STATE_PROPAGATOR_H