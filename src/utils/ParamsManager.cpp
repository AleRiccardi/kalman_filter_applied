#include "utils/ParamsManager.h"

#include <ros/ros.h>

ParamsManager::ParamsManager(ros::NodeHandle& nh) {
  // ---------------------------------------------------------------------------
  // General params

  nh.param<double>("rate_gt", rate_gt, 5);
  nh.param<double>("rate_gps", rate_gps, 5);
  nh.param<double>("rate_radar", rate_radar, 5);

  // ---------------------------------------------------------------------------
  // GPS Noise Matrix

  std::vector<double> gps_noise_v = {gps_noise(0), gps_noise(1), gps_noise(2)};
  nh.param<std::vector<double>>("gps_noise", gps_noise_v, gps_noise_v);
  assert(gps_noise_v.size() == 3);
  gps_noise << gps_noise_v[0], gps_noise_v[1], gps_noise_v[2];

  // ---------------------------------------------------------------------------
  // ROS TOPICS (& replated)

  // Frame ID
  nh.param<std::string>("frame_id", frame_id, "global");
  ROS_INFO("Topic frame_id is: %s", frame_id.c_str());

  // GT pose
  nh.param<std::string>("topic_gt", topic_gt, "/kf_applied/gt");
  ROS_INFO("Topic gt is: %s", topic_gt.c_str());
  nh.param<std::string>("topic_gt_path", topic_gt_path, "/kf_applied/gt_path");
  ROS_INFO("Topic gt path is: %s", topic_gt_path.c_str());

  // ESTIMATION pose
  nh.param<std::string>("topic_estimation", topic_estimation,
                        "/kf_applied/estimation");
  ROS_INFO("Topic estimation is: %s", topic_estimation.c_str());
  nh.param<std::string>("topic_estimation_path", topic_estimation_path,
                        "/kf_applied/estimation_path");
  ROS_INFO("Topic estimation path is: %s", topic_estimation_path.c_str());

  // GPS pose
  nh.param<std::string>("topic_gps", topic_gps, "/kf_applied/gps");
  ROS_INFO("Topic gps is: %s", topic_gps.c_str());
  nh.param<std::string>("topic_gps_path", topic_gps_path,
                        "/kf_applied/gps_path");
  ROS_INFO("Topic gps path is: %s", topic_gps_path.c_str());

  // RADAR pose
  nh.param<std::string>("topic_radar", topic_radar, "/kf_applied/gps");
  ROS_INFO("Topic radar is: %s", topic_radar.c_str());

  PrintNoise();
}
