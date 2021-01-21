#ifndef KF_APPLIED_PARAMSMANAGER_H
#define KF_APPLIED_PARAMSMANAGER_H

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

  double rate_gt;
  double rate_gps;
  double rate_radar;

  std::string frame_id;
  std::string topic_gt;
  std::string topic_gt_path;
  std::string topic_estimation;
  std::string topic_estimation_path;
  std::string topic_gps;
  std::string topic_gps_path;
  std::string topic_radar;

  // Noise vector
  Eigen::Vector3d noise_gps = {0, 0, 0};
  Eigen::Vector3d noise_radar = {0, 0, 0};

  Eigen::Matrix<double, 3, 1> init_pose;
  Eigen::Matrix<double, 3, 1> init_pose_radar;

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

#endif  // KF_APPLIED_MANAGEROPTIONS_H