#ifndef KF_APPLIED_SIM_MANAGER_H
#define KF_APPLIED_SIM_MANAGER_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <time.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "simulation/StateManager.h"
#include "utils/ParamsManager.h"

struct Character {
  char value;
  std::string info;
};

class Command {
 public:
  static void print_cmd();

  static Character forward_;
  static Character backward_;
  static Character left_;
  static Character right_;
  static Character up_;
  static Character down_;
  static Character record_;
};

class SimManager {
 public:
  SimManager(ros::NodeHandle nh, ParamsManager* params);
  ~SimManager() = default;

  bool user_control();
  void pub_state();
  void propagate();
  void start_record();
  void stop_record();
  void kill();

  double dt = 0.2;

 private:
  StateManager* state_m_;

  rosbag::Bag bag_;
  ParamsManager* params_;

  ros::Publisher pub_gt_pose_;
  ros::Publisher pub_gt_path_;
  ros::Publisher pub_gps_pose_;
  ros::Publisher pub_gps_path_;
  ros::Publisher pub_radar_pose_;

  std::vector<geometry_msgs::PoseStamped> poses_gt_;
  std::vector<geometry_msgs::PoseStamped> poses_gps_;

  const double ACC = 0.05;
  Eigen::Matrix<double, 3, 1> acceleration_;

  int gt_count_ = 0;
  int gps_count_ = 0;

  bool recording_;
};

#endif  // KF_APPLIED_SIM_MANAGER_H