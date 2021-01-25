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

SimManager::SimManager(ros::NodeHandle nh, ParamsManager *params) {
  params_ = params;
  state_m_ = new StateManager(nh, params);

  // Get sensors frequency
  period_gt_ = ros::Duration(1 / params_->frequency_gt);
  period_gps_ = ros::Duration(1 / params_->frequency_gps);
  period_radar_ = ros::Duration(1 / params_->frequency_radar);

  ros::Time time = ros::Time::now();
  time_gt_ = time;
  time_gps_ = time;
  time_radar_ = time;

  Command::print_cmd();

  // ---------------------------------------------------------------------------
  // TOPICS

  // Ground Truth
  pub_pose_gt_ = nh.advertise<geometry_msgs::PoseStamped>(params_->topic_gt, 2);
  pub_path_gt_ = nh.advertise<nav_msgs::Path>(params_->topic_gt_path, 2);

  // GOS
  std::string topic_gps;
  pub_pose_gps_ =
      nh.advertise<geometry_msgs::PoseStamped>(params_->topic_gps, 2);
  pub_path_gps_ = nh.advertise<nav_msgs::Path>(params_->topic_gps_path, 2);

  // Radar
  pub_radar_ =
      nh.advertise<geometry_msgs::Vector3Stamped>(params_->topic_radar, 2);
}

bool SimManager::user_control() {
  acceleration_ = Eigen::Vector3d::Zero();

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

void SimManager::publisher() {
  // Get sensor state
  Eigen::Vector3d gt = state_m_->get_gt();
  Eigen::Vector3d gps = state_m_->get_gps();
  Eigen::Vector3d radar = state_m_->get_radar();

  ros::Time time = ros::Time::now();

  if ((time - time_gt_) > period_gt_) {
    pub_pose_stamped(gt, time, count_gt_++, poses_gt_, pub_pose_gt_,
                     pub_path_gt_);

    time_gt_ = time;
  }

  if ((time - time_gps_) > period_gps_) {
    pub_pose_stamped(gps, time, count_radar_++, poses_gps_, pub_pose_gps_,
                     pub_path_gps_);

    time_gps_ = time;
  }

  if ((time - time_radar_) > period_radar_) {
    pub_radar_scan(radar, time, count_radar_++, pub_radar_);

    time_radar_ = time;
  }
}

void SimManager::pub_pose_stamped(
    Eigen::Vector3d state, ros::Time time, int seq,
    std::vector<geometry_msgs::PoseStamped> &poses, ros::Publisher &pub_pose,
    ros::Publisher &pub_path) {
  // ---------------------------------------------------------------------------
  // Pose

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = time;
  pose.header.seq = seq;
  pose.header.frame_id = params_->frame_id;
  pose.pose.position.x = state(0, 0);
  pose.pose.position.y = state(1, 0);
  pose.pose.position.z = state(2, 0);
  pub_pose.publish(pose);

  // ---------------------------------------------------------------------------
  // Path

  poses.push_back(pose);
  nav_msgs::Path arr_poses_noise;
  arr_poses_noise.header.stamp = time;
  arr_poses_noise.header.seq = seq;
  arr_poses_noise.header.frame_id = params_->frame_id;
  for (size_t i = 0; i < poses.size();
       i += std::floor(poses.size() / 16384.0) + 1) {
    arr_poses_noise.poses.push_back(poses.at(i));
  }
  pub_path.publish(arr_poses_noise);

  if (recording_) {
    bag_.write(pub_pose.getTopic(), pose.header.stamp, pose);
  }
}

void SimManager::pub_radar_scan(Eigen::Vector3d state, ros::Time time, int seq,
                                ros::Publisher &pub_radar) {
  geometry_msgs::Vector3Stamped radar;
  radar.header.stamp = time;
  radar.header.seq = seq;
  radar.header.frame_id = params_->frame_id;
  radar.vector.x = state(0, 0);
  radar.vector.y = state(1, 0);
  radar.vector.z = state(2, 0);

  // TODO: publish lines showing radar scans

  pub_radar.publish(radar);

  if (recording_) {
    bag_.write(pub_radar.getTopic(), radar.header.stamp, radar);
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