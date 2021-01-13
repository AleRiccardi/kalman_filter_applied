#ifndef KF_APPLIED_MANAGEROPTIONS_H
#define KF_APPLIED_MANAGEROPTIONS_H

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <vector>

class ParamsManager {
 public:
  ParamsManager(ros::NodeHandle& nh);
  ~ParamsManager() = default;

  // NOISE ============================
  Eigen::Vector3d gps_noise = {0.15, 0.15, 0.05};
  /**
   * @brief This function will print out all noise parameters loaded.
   * This allows for visual checking that everything was loaded properly from
   * ROS parsers.
   */
  void PrintNoise() const {
    ROS_INFO("NOISE PARAMETERS:");
    ROS_INFO_STREAM("\n" << gps_noise);
  }

 private:
  // abc
};

#endif  // KF_APPLIED_MANAGEROPTIONS_H