#ifndef KF_APPLIED_SIM_MANAGER_H
#define KF_APPLIED_SIM_MANAGER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <time.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "simulator/StatePropagator.h"
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
  void publisher();
  void pub_pose_stamped(Eigen::Vector3d state, ros::Time time, int seq,
                        std::vector<geometry_msgs::PoseStamped>& poses,
                        ros::Publisher& pub_pose, ros::Publisher& pub_path);
  void pub_radar_scan(Eigen::Vector3d state, ros::Time time, int seq,
                      ros::Publisher& pub_radar);
  void propagate();
  void start_record();
  void stop_record();
  void kill();

  double dt = 0.2;

 private:
  ParamsManager* params_;
  StatePropagator* state_m_;

  rosbag::Bag bag_;

  ros::Publisher pub_pose_gt_;
  ros::Publisher pub_path_gt_;
  ros::Publisher pub_pose_gps_;
  ros::Publisher pub_path_gps_;
  ros::Publisher pub_radar_;

  std::vector<geometry_msgs::PoseStamped> poses_gt_;
  std::vector<geometry_msgs::PoseStamped> poses_gps_;

  const double ACC = 0.05;
  Eigen::Vector3d acceleration_;

  int count_gt_ = 0;
  int count_gps_ = 0;
  int count_radar_ = 0;

  ros::Duration period_gt_;
  ros::Duration period_gps_;
  ros::Duration period_radar_;

  ros::Time time_gt_;
  ros::Time time_gps_;
  ros::Time time_radar_;

  bool recording_;
};

#endif  // KF_APPLIED_SIM_MANAGER_H