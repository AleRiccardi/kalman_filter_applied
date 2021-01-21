#include "simulation/SimManager.h"

#include <conio.h>
#include <ros/package.h>

#include <string>

Character Command::forward_ = {'i', "Forward"};
Character Command::backward_ = {'k', "Backward"};
Character Command::left_ = {'j', "Left"};
Character Command::right_ = {'l', "Right"};
Character Command::up_ = {'u', "Up"};
Character Command::down_ = {'o', "Down"};
Character Command::record_ = {'r', "Record"};

void Command::print_cmd() {
  std::cout << std::endl;
  ROS_INFO("Use the following commands for moving the agent: ");
  ROS_INFO("%c - %s", Command::forward_.value, Command::forward_.info.c_str());
  ROS_INFO("%c - %s", Command::backward_.value,
           Command::backward_.info.c_str());
  ROS_INFO("%c - %s", Command::left_.value, Command::left_.info.c_str());
  ROS_INFO("%c - %s", Command::right_.value, Command::right_.info.c_str());
  ROS_INFO("%c - %s", Command::up_.value, Command::up_.info.c_str());
  ROS_INFO("%c - %s", Command::down_.value, Command::down_.info.c_str());
  ROS_INFO("%c - %s", Command::record_.value, Command::record_.info.c_str());
  std::cout << std::endl;
}

SimManager::SimManager(ros::NodeHandle nh, ParamsManager* params) {
  params_ = params;

  state_m_ = new StateManager(nh, params);

  Command::print_cmd();

  // ---------------------------------------------------------------------------
  // TOPICS

  // ground truth
  pub_gt_pose_ = nh.advertise<geometry_msgs::PoseStamped>(params_->topic_gt, 2);
  pub_gt_path_ = nh.advertise<nav_msgs::Path>(params_->topic_gt_path, 2);

  // gps
  std::string topic_gps;
  pub_gps_pose_ =
      nh.advertise<geometry_msgs::PoseStamped>(params_->topic_gps, 2);
  pub_gps_path_ = nh.advertise<nav_msgs::Path>(params_->topic_gps_path, 2);

  // radar
  pub_radar_pose_ =
      nh.advertise<geometry_msgs::PoseStamped>(params_->topic_radar, 2);
}

bool SimManager::user_control() {
  acceleration_ = Eigen::Matrix<double, 3, 1>::Zero();

  if (kbhit()) {
    bool known_cmd = true;
    // Stores the pressed key in ch
    // TODO: check conversion
    char u_input = int(getch());

    if (u_input == Command::forward_.value) {
      acceleration_(0, 0) = ACC;
    } else if (u_input == Command::backward_.value) {
      acceleration_(0, 0) = -ACC;
    } else if (u_input == Command::left_.value) {
      acceleration_(1, 0) = ACC;
    } else if (u_input == Command::right_.value) {
      acceleration_(1, 0) = -ACC;
    } else if (u_input == Command::up_.value) {
      acceleration_(2, 0) = ACC;
    } else if (u_input == Command::down_.value) {
      acceleration_(2, 0) = -ACC;
    } else if (u_input == Command::record_.value) {
      if (!recording_) {
        start_record();
      } else {
        stop_record();
      }
    } else {
      known_cmd = false;
    }
    return known_cmd;
  }
  return false;
}

void SimManager::propagate() { state_m_->propagate(acceleration_); }

void SimManager::pub_state() {
  Eigen::Matrix<double, 3, 1> gt = state_m_->get_gt();
  Eigen::Matrix<double, 3, 1> gps = state_m_->get_gps();

  // Create pose (note we use the bag time)
  geometry_msgs::PoseStamped pose_gt;
  pose_gt.header.stamp = ros::Time::now();
  pose_gt.header.seq = gt_count_;
  pose_gt.header.frame_id = params_->frame_id;
  pose_gt.pose.position.x = gt(0, 0);
  pose_gt.pose.position.y = gt(1, 0);
  pose_gt.pose.position.z = gt(2, 0);

  pub_gt_pose_.publish(pose_gt);

  // ---------------------------------------------------------------------------

  // Create pose (note we use the bag time)
  geometry_msgs::PoseStamped pose_gps;
  pose_gps.header.stamp = ros::Time::now();
  pose_gps.header.seq = gps_count_;
  pose_gps.header.frame_id = params_->frame_id;
  pose_gps.pose.position.x = gps(0, 0);
  pose_gps.pose.position.y = gps(1, 0);
  pose_gps.pose.position.z = gps(2, 0);

  pub_gps_pose_.publish(pose_gps);

  // ---------------------------------------------------------------------------

  // Append to our pose vectors
  poses_gt_.push_back(pose_gt);
  poses_gps_.push_back(pose_gps);

  // Create our path
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arr_poses_gt;
  arr_poses_gt.header.stamp = pose_gt.header.stamp;
  arr_poses_gt.header.seq = gt_count_++;
  arr_poses_gt.header.frame_id = params_->frame_id;
  for (size_t i = 0; i < poses_gt_.size();
       i += std::floor(poses_gt_.size() / 16384.0) + 1) {
    arr_poses_gt.poses.push_back(poses_gt_.at(i));
  }
  pub_gt_path_.publish(arr_poses_gt);

  // ---------------------------------------------------------------------------

  nav_msgs::Path arr_poses_noise;
  arr_poses_noise.header.stamp = pose_gps.header.stamp;
  arr_poses_noise.header.seq = gps_count_++;
  arr_poses_noise.header.frame_id = params_->frame_id;
  for (size_t i = 0; i < poses_gps_.size();
       i += std::floor(poses_gps_.size() / 16384.0) + 1) {
    arr_poses_noise.poses.push_back(poses_gps_.at(i));
  }
  pub_gps_path_.publish(arr_poses_noise);

  if (recording_) {
    bag_.write(pub_gt_pose_.getTopic(), pose_gt.header.stamp, pose_gt);
    bag_.write(pub_gps_pose_.getTopic(), pose_gps.header.stamp, pose_gps);
  }
}

void SimManager::start_record() {
  recording_ = true;
  // TODO(alericcardi): get name from ros
  std::string path_pkg = ros::package::getPath("kf_applied") + "/data/";
  std::string file_name =
      std::to_string(static_cast<int>(ros::Time::now().toSec())) + ".bag";
  std::string path_pname = path_pkg + file_name;
  ROS_INFO("Recording started");
  bag_.open(path_pname, rosbag::bagmode::Write);
}

void SimManager::stop_record() {
  ROS_INFO("Recording stopped");
  recording_ = false;
  bag_.close();
}

void SimManager::kill() {
  if (recording_) {
    recording_ = false;
    bag_.close();
  }
}