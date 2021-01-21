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

  std::string frame_id;
  std::string topic_gt;
  std::string topic_gps;
  std::string topic_radar;
  std::string topic_estimation;
  // Noise vector
  Eigen::Vector3d gps_noise = {0.15, 0.15, 0.05};

  /**
   * @brief This function will print out all noise parameters loaded.
   * This allows for visual checking that everything was loaded properly from
   * ROS parsers.
   */
  void PrintNoise() const {
    printf(GREEN "\t- NOISE PARAMETERS - \n" RESET);
    std::cout << gps_noise << std::endl;
  }

 private:
  // abc
};

#endif  // KF_APPLIED_MANAGEROPTIONS_H