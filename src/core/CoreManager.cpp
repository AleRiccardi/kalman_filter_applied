#include <core/CoreManager.h>
#include <utils/colors.h>

#include <utility>

CoreManager::CoreManager(ros::NodeHandle nh, ParamsManager* params) {
  params_ = params;
  kf_ = new KalmanFilter(params);

  pub_estimation_ =
      nh.advertise<geometry_msgs::PoseStamped>(params_->topic_estimation, 2);
  ROS_INFO("Publishing: %s", pub_estimation_.getTopic().c_str());

  pub_estimation_path_ =
      nh.advertise<nav_msgs::Path>(params_->topic_estimation + "_path", 2);
  ROS_INFO("Publishing: %s", pub_estimation_path_.getTopic().c_str());

  pub_gt_pose_ = nh.advertise<geometry_msgs::PoseStamped>(params_->topic_gt, 2);
  ROS_INFO("Publishing: %s", pub_gt_pose_.getTopic().c_str());

  pub_gt_path = nh.advertise<nav_msgs::Path>(params_->topic_gt + "_path", 2);
  ROS_INFO("Publishing: %s", pub_gt_path.getTopic().c_str());
}

void CoreManager::feed_m_gt(double timestamp, Eigen::Vector3d pose) {
  GPS_DATA data;
  data.timestamp = timestamp;
  data.pose = std::move(pose);
  gt_data_.emplace_back(data);
  // Update current time
  curr_time_ = timestamp;
}

void CoreManager::feed_m_gps(double timestamp, Eigen::Vector3d pose) {
  GPS_DATA data;
  data.timestamp = timestamp;
  data.pose = std::move(pose);
  gps_data_.emplace_back(data);

  // Update current time
  curr_time_ = timestamp;
}

void CoreManager::feed_m_radar(double timestamp, Eigen::Vector3d beam) {
  RADAR_DATA data;
  data.timestamp = timestamp;
  data.beam = std::move(beam);
  radar_data_.emplace_back(data);

  // Update current time
  curr_time_ = timestamp;
}

void CoreManager::state_estimation() {
  if (!is_initialized_) {
    is_initialized_ = initialize();
    return;
  }

  // std::vector<GPS_DATA> gps_v;
  // pop_gps(gps_v);
  // if (!gps_v.empty()) {
  //   for (uint i = 0; i < gps_v.size(); i++) {
  //     // kf_->propagation(gps_v.at(i).timestamp);
  //     kf_->correction_gps(gps_v.at(i));
  //   }
  // }

  std::vector<RADAR_DATA> radar_v;
  pop_radar(radar_v);
  if (!radar_v.empty()) {
    for (uint i = 0; i < radar_v.size(); i++) {
      kf_->propagation(radar_v.at(i).timestamp);
      kf_->correction_radar(radar_v.at(i));
    }
  }
}

bool CoreManager::initialize() {
  // Required 2 GPS measurements for init
  if (gps_data_.size() < 2) {
    return false;
  }

  GPS_DATA gps1 = gps_data_[0];
  GPS_DATA gps2 = gps_data_[1];
  // Erase the first element
  gps_data_.erase(gps_data_.begin());

  kf_->initialize(gps1, gps2);

  return true;
}

bool CoreManager::pop_gps(std::vector<GPS_DATA>& gps_v) {
  if (gps_data_.empty()) {
    return false;
  }
  std::cout << gps_data_.size() << std::endl;

  gps_v.emplace_back(gps_data_[0]);
  gps_data_.erase(gps_data_.begin());
  return true;
}

bool CoreManager::pop_radar(std::vector<RADAR_DATA>& radar_v) {
  if (radar_data_.empty()) {
    return false;
  }

  radar_v.emplace_back(radar_data_[0]);
  radar_data_.erase(radar_data_.begin());
  return true;
}

void CoreManager::display() {
  // TODO: publish the correct ground truth pose based on the time
  // of the state estimation and not based on the last stored ground truth.
  if (gt_data_.empty()) return;

  Eigen::MatrixXd state = kf_->GetState();
  Eigen::MatrixXd gt = gt_data_[gt_data_.size() - 1].pose;

  // Create pose (note we use the bag time)
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.seq = count_poses_;
  pose.header.frame_id = params_->frame_id;
  pose.pose.position.x = state(0, 0);
  pose.pose.position.y = state(1, 0);
  pose.pose.position.z = state(2, 0);

  pub_estimation_.publish(pose);

  // ---------------------------------------------------------------------------

  // Create pose (note we use the bag time)
  geometry_msgs::PoseStamped pose_gt;
  pose_gt.header.stamp = ros::Time::now();
  pose_gt.header.seq = count_poses_;
  pose_gt.header.frame_id = params_->frame_id;
  pose_gt.pose.position.x = gt(0, 0);
  pose_gt.pose.position.y = gt(1, 0);
  pose_gt.pose.position.z = gt(2, 0);

  pub_gt_pose_.publish(pose_gt);

  // ---------------------------------------------------------------------------

  // Append to our pose vectors
  poses_.push_back(pose);
  poses_gt_.push_back(pose_gt);

  // Create our path
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arr_poses;
  arr_poses.header.stamp = pose.header.stamp;
  arr_poses.header.seq = count_poses_;
  arr_poses.header.frame_id = params_->frame_id;
  for (size_t i = 0; i < poses_.size();
       i += std::floor(poses_.size() / 16384.0) + 1) {
    arr_poses.poses.push_back(poses_.at(i));
  }
  pub_estimation_path_.publish(arr_poses);

  // ---------------------------------------------------------------------------

  nav_msgs::Path arr_poses_gt;
  arr_poses_gt.header.stamp = pose_gt.header.stamp;
  arr_poses_gt.header.seq = count_poses_;
  arr_poses_gt.header.frame_id = params_->frame_id;
  for (size_t i = 0; i < poses_gt_.size();
       i += std::floor(poses_gt_.size() / 16384.0) + 1) {
    arr_poses_gt.poses.push_back(poses_gt_.at(i));
  }
  pub_gt_path.publish(arr_poses_gt);

  // Move forward in time
  count_poses_++;
}