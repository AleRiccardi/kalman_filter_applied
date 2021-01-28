#include "utils/ParamsManager.h"

#include <ros/ros.h>

ParamsManager::ParamsManager(ros::NodeHandle& nh) {
  // ---------------------------------------------------------------------------
  // General params

  nh.param<double>("frequency_gt", frequency_gt, 5);
  nh.param<double>("frequency_gps", frequency_gps, 5);
  nh.param<double>("frequency_radar", frequency_radar, 5);

  // ---------------------------------------------------------------------------
  // Constant process acceleration vector

  std::vector<double> init_acceleration_v = {
      init_acceleration(0), init_acceleration(1), init_acceleration(2)};
  nh.param<std::vector<double>>("init_acceleration", init_acceleration_v,
                                init_acceleration_v);
  init_acceleration << init_acceleration_v[0], init_acceleration_v[1],
      init_acceleration_v[2];

  // ---------------------------------------------------------------------------
  // Process noise vector

  std::vector<double> noise_process_v = {noise_process(0), noise_process(1),
                                         noise_process(2)};
  nh.param<std::vector<double>>("noise_process", noise_process_v,
                                noise_process_v);
  noise_process << noise_process_v[0], noise_process_v[1], noise_process_v[2];

  // ---------------------------------------------------------------------------
  // GPS noise vector

  std::vector<double> noise_gps_v = {noise_gps(0), noise_gps(1), noise_gps(2)};
  nh.param<std::vector<double>>("noise_gps", noise_gps_v, noise_gps_v);
  noise_gps << noise_gps_v[0], noise_gps_v[1], noise_gps_v[2];

  // ---------------------------------------------------------------------------
  // Radar noise vector

  std::vector<double> noise_radar_v = {noise_radar(0), noise_radar(1),
                                       noise_radar(2)};
  nh.param<std::vector<double>>("noise_radar", noise_radar_v, noise_radar_v);
  noise_radar << noise_radar_v[0], noise_radar_v[1], noise_radar_v[2];

  // ---------------------------------------------------------------------------
  // Initial pose

  std::vector<double> init_pose_v = {0, 0, 0};
  nh.param<std::vector<double>>("init_pose", init_pose_v, init_pose_v);
  assert(init_pose_v.size() == 3);
  init_pose << init_pose_v[0], init_pose_v[1], init_pose_v[2];

  // ---------------------------------------------------------------------------
  // Initial pose of the radar

  std::vector<double> init_pose_radar_v = {0, 0, 0};
  nh.param<std::vector<double>>("init_pose_radar", init_pose_radar_v,
                                init_pose_radar_v);
  assert(init_pose_radar_v.size() == 3);
  init_pose_radar << init_pose_radar_v[0], init_pose_radar_v[1],
      init_pose_radar_v[2];

  // ---------------------------------------------------------------------------
  // ROS TOPICS (& replated)

  // Frame ID
  nh.param<std::string>("frame_id", frame_id, "global");
  ROS_INFO("Topic frame_id is: %s", frame_id.c_str());

  // GT pose
  nh.param<std::string>("topic_gt", topic_gt, "/kfa/gt");
  ROS_INFO("Topic gt is: %s", topic_gt.c_str());
  nh.param<std::string>("topic_gt_path", topic_gt_path, "/kfa/gt_path");
  ROS_INFO("Topic gt path is: %s", topic_gt_path.c_str());

  // ESTIMATION pose
  nh.param<std::string>("topic_estimation", topic_estimation,
                        "/kfa/estimation");
  ROS_INFO("Topic estimation is: %s", topic_estimation.c_str());
  nh.param<std::string>("topic_estimation_path", topic_estimation_path,
                        "/kfa/estimation_path");
  ROS_INFO("Topic estimation path is: %s", topic_estimation_path.c_str());

  // GPS pose
  nh.param<std::string>("topic_gps", topic_gps, "/kfa/gps");
  ROS_INFO("Topic gps is: %s", topic_gps.c_str());
  nh.param<std::string>("topic_gps_path", topic_gps_path, "/kfa/gps_path");
  ROS_INFO("Topic gps path is: %s", topic_gps_path.c_str());

  // Radar pose
  nh.param<std::string>("topic_radar", topic_radar, "/kfa/gps");
  ROS_INFO("Topic radar is: %s", topic_radar.c_str());

  PrintNoise();
}
