#ifndef KFA_PARAMS_MANAGER_H
#define KFA_PARAMS_MANAGER_H

#include <ros/ros.h>
#include <utils/colors.h>

#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <vector>

class ParamsManager {
 public:
  ParamsManager(ros::NodeHandle& nh);
  ~ParamsManager() = default;

  double frequency_gt;
  double frequency_gps;
  double frequency_radar;

  std::string frame_id;
  std::string topic_gt;
  std::string topic_gt_path;
  std::string topic_estimation;
  std::string topic_estimation_path;
  std::string topic_gps;
  std::string topic_gps_path;
  std::string topic_radar;

  Eigen::Vector3d init_acceleration = {0, 0, 0};

  // Noise vector
  Eigen::Vector3d noise_process = {0, 0, 0};
  Eigen::Vector3d noise_gps = {0, 0, 0};
  Eigen::Vector3d noise_radar = {0, 0, 0};

  Eigen::Vector3d init_pose;
  Eigen::Vector3d init_pose_radar;

  /**
   * @brief This function will print out all noise parameters loaded.
   * This allows for visual checking that everything was loaded properly from
   * ROS parsers.
   */
  void PrintNoise() const {
    printf(GREEN "\t- NOISE PARAMETERS - \n" RESET);
    std::cout << noise_gps << std::endl;
  }

 private:
  // abc
};

#endif  // KFA_PARAMS_MANAGER_H