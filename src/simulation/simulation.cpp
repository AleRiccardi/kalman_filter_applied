#include "simulation/simulation.h"

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

Simulation::Simulation(ros::NodeHandle nh) {
  // ---------------------------------------------------------------------------
  // TOPICS

  // GT pose
  std::string topic_gt;
  nh.param<std::string>("topic_gt", topic_gt, "/kf_applied/gt");
  pub_gt_pose_ = nh.advertise<geometry_msgs::PoseStamped>(topic_gt.c_str(), 2);
  ROS_INFO("Topic gt is: %s", topic_gt.c_str());

  // GT path
  std::string topic_gt_path_ = topic_gt + "_path";
  pub_gt_path_ = nh.advertise<nav_msgs::Path>(topic_gt_path_, 2);
  ROS_INFO("Topic gt path is: %s", pub_gt_path_.getTopic().c_str());

  // GPS pose
  std::string topic_gps;
  nh.param<std::string>("topic_gps", topic_gps, "/kf_applied/gps");
  pub_gps_pose_ =
      nh.advertise<geometry_msgs::PoseStamped>(topic_gps.c_str(), 2);
  ROS_INFO("Topic gps is: %s", topic_gps.c_str());

  // GPS path
  std::string topic_gps_path_ = topic_gps + "_path";
  pub_gps_path_ = nh.advertise<nav_msgs::Path>(topic_gps_path_, 2);
  ROS_INFO("Topic gps path is: %s", pub_gps_path_.getTopic().c_str());

  // ---------------------------------------------------------------------------
  // PARAMS

  // Initialize random seed
  std::srand(time(nullptr));

  state_.setZero();
  acc_.setZero();
  // TODO: load from launch
  noise_gps_ = Eigen::Vector3d(0.15, 0.15, 0.05);
  std::cout << state_ << std::endl;
  _F << 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  Command::print_cmd();
}

bool Simulation::user_control() {
  acc_ = Eigen::Matrix<double, 3, 1>::Zero();

  if (kbhit()) {
    bool known_cmd = true;
    // Stores the pressed key in ch
    // TODO: check conversion
    char u_input = int(getch());

    if (u_input == Command::forward_.value) {
      acc_(0) = ACC;
    } else if (u_input == Command::backward_.value) {
      acc_(0) = -ACC;
    } else if (u_input == Command::left_.value) {
      acc_(1) = ACC;
    } else if (u_input == Command::right_.value) {
      acc_(1) = -ACC;
    } else if (u_input == Command::up_.value) {
      acc_(2) = ACC;
    } else if (u_input == Command::down_.value) {
      acc_(2) = -ACC;
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

void Simulation::propagate() {
  // _state evolution
  for (int i = 0; i <= _F.rows() - 1; i++) {
    double val = _F.row(i).dot(state_);
    state_(i, 0) = val;
  }

  state_.tail(3) += acc_;

  apply_noise();
}

void Simulation::pub_state() {
  // Create pose (note we use the bag time)
  geometry_msgs::PoseStamped pose_gt;
  pose_gt.header.stamp = ros::Time::now();
  pose_gt.header.seq = poses_count_;
  pose_gt.header.frame_id = "global";
  pose_gt.pose.position.x = state_(0, 0);
  pose_gt.pose.position.y = state_(1, 0);
  pose_gt.pose.position.z = state_(2, 0);

  pub_gt_pose_.publish(pose_gt);

  // ---------------------------------------------------------------------------

  // Create pose (note we use the bag time)
  geometry_msgs::PoseStamped pose_noise;
  pose_noise.header.stamp = ros::Time::now();
  pose_noise.header.seq = poses_count_;
  pose_noise.header.frame_id = "global";
  pose_noise.pose.position.x = state_gps_(0, 0);
  pose_noise.pose.position.y = state_gps_(1, 0);
  pose_noise.pose.position.z = state_gps_(2, 0);

  pub_gps_pose_.publish(pose_noise);

  // ---------------------------------------------------------------------------

  // Append to our pose vectors
  poses_gps_.push_back(pose_gt);
  poses_noise_.push_back(pose_noise);

  // Create our path
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arr_poses_gt;
  arr_poses_gt.header.stamp = pose_gt.header.stamp;
  arr_poses_gt.header.seq = poses_count_;
  arr_poses_gt.header.frame_id = "global";
  for (size_t i = 0; i < poses_gps_.size();
       i += std::floor(poses_gps_.size() / 16384.0) + 1) {
    arr_poses_gt.poses.push_back(poses_gps_.at(i));
  }
  pub_gt_path_.publish(arr_poses_gt);

  // ---------------------------------------------------------------------------

  nav_msgs::Path arr_poses_noise;
  arr_poses_noise.header.stamp = pose_noise.header.stamp;
  arr_poses_noise.header.seq = poses_count_;
  arr_poses_noise.header.frame_id = "global";
  for (size_t i = 0; i < poses_noise_.size();
       i += std::floor(poses_noise_.size() / 16384.0) + 1) {
    arr_poses_noise.poses.push_back(poses_noise_.at(i));
  }
  pub_gps_path_.publish(arr_poses_noise);

  // Move forward in time
  poses_count_++;

  if (recording_) {
    bag_.write(pub_gt_pose_.getTopic(), pose_gt.header.stamp, pose_gt);
    bag_.write(pub_gps_pose_.getTopic(), pose_noise.header.stamp, pose_noise);
  }
}

void Simulation::apply_noise() {
  Eigen::Vector3d rands;

  rands(0, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(1, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;
  rands(2, 0) = static_cast<double>((std::rand() % 200) - 100) / 100;

  // TODO(alericcardi): inline multiplication
  Eigen::Vector3d noise;
  noise(0) = noise_gps_(0) * rands(0);
  noise(1) = noise_gps_(1) * rands(1);
  noise(2) = noise_gps_(2) * rands(2);

  state_gps_ = state_;
  state_gps_.head(3) = state_.head(3) + noise;
}

void Simulation::start_record() {
  recording_ = true;
  // TODO(alericcardi): get name from ros
  std::string path_pkg = ros::package::getPath("kf_applied") + "/data/";
  std::string file_name =
      std::to_string(static_cast<int>(ros::Time::now().toSec())) + ".bag";
  std::string path_pname = path_pkg + file_name;
  ROS_INFO("Recording started");
  bag_.open(path_pname, rosbag::bagmode::Write);
}

void Simulation::stop_record() {
  ROS_INFO("Recording stopped");
  recording_ = false;
  bag_.close();
}

void Simulation::kill() {
  if (recording_) {
    recording_ = false;
    bag_.close();
  }
}