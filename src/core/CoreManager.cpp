#include <core/CoreManager.h>
#include <utils/colors.h>

#include <utility>

CoreManager::CoreManager(ros::NodeHandle nh, ParamsManager* params) {
  params_ = params;
  kf_ = new KalmanFilter(params);

  pub_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/kf_applied/pose", 2);
  ROS_INFO("Publishing: %s", pub_pose_.getTopic().c_str());

  pub_path_ = nh.advertise<nav_msgs::Path>("/kf_applied/path", 2);
  ROS_INFO("Publishing: %s", pub_path_.getTopic().c_str());

  pub_pose_gt_ =
      nh.advertise<geometry_msgs::PoseStamped>("/kf_applied/pose_gt", 2);
  ROS_INFO("Publishing: %s", pub_pose_gt_.getTopic().c_str());

  pub_path_gt_ = nh.advertise<nav_msgs::Path>("/kf_applied/path_gt", 2);
  ROS_INFO("Publishing: %s", pub_path_gt_.getTopic().c_str());
}

void CoreManager::FeedMeasurementGPS(double timestamp, Eigen::Vector3d pose) {
  GPSDATA data;
  data.timestamp = timestamp;
  data.pose = std::move(pose);

  gps_data_.emplace_back(data);
}

void CoreManager::FeedGT(double timestamp, Eigen::Vector3d pose) {
  GPSDATA data;
  data.timestamp = timestamp;
  data.pose = std::move(pose);

  gt_data_.emplace_back(data);
}

void CoreManager::StateEstimation() {
  if (!is_initialized_) {
    is_initialized_ = Initialize();
    return;
  }

  GPSDATA gps;
  if (!PopMeasurement(gps)) {
    printf(YELLOW "No GPS measurements stored \n" RESET);
    return;
  }

  kf_->Propagation(gps.timestamp);
  kf_->Correction(gps.pose);
}

bool CoreManager::Initialize() {
  // Required 2 GPS measurements for init
  if (gps_data_.size() < 2) {
    return false;
  }

  GPSDATA gps1 = gps_data_[0];
  GPSDATA gps2 = gps_data_[1];
  // Erase the first element
  gps_data_.erase(gps_data_.begin());

  kf_->Initialize(gps1, gps2);

  return true;
}

bool CoreManager::PopMeasurement(GPSDATA& gps) {
  if (gps_data_.empty()) {
    return false;
  }

  gps = gps_data_[0];
  gps_data_.erase(gps_data_.begin());
  return true;
}

void CoreManager::Display() {
  // TODO: publish the correct ground truth pose based on the time
  // of the state estimation and not based on the last stored ground truth.
  if (gt_data_.empty()) return;

  Eigen::MatrixXd state = kf_->GetState();
  Eigen::MatrixXd gt = gt_data_[gt_data_.size() - 1].pose;

  // Create pose (note we use the bag time)
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.seq = poses_count_;
  pose.header.frame_id = "global";
  pose.pose.position.x = state(0, 0);
  pose.pose.position.y = state(1, 0);
  pose.pose.position.z = state(2, 0);

  pub_pose_.publish(pose);

  // =======================================================

  // Create pose (note we use the bag time)
  geometry_msgs::PoseStamped pose_gt;
  pose_gt.header.stamp = ros::Time::now();
  pose_gt.header.seq = poses_count_;
  pose_gt.header.frame_id = "global";
  pose_gt.pose.position.x = gt(0, 0);
  pose_gt.pose.position.y = gt(1, 0);
  pose_gt.pose.position.z = gt(2, 0);

  pub_pose_gt_.publish(pose_gt);

  //=========================================================
  //=========================================================

  // Append to our pose vectors
  poses_.push_back(pose);
  poses_gt_.push_back(pose_gt);

  // Create our path
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arr_poses;
  arr_poses.header.stamp = pose.header.stamp;
  arr_poses.header.seq = poses_count_;
  arr_poses.header.frame_id = "global";
  for (size_t i = 0; i < poses_.size();
       i += std::floor(poses_.size() / 16384.0) + 1) {
    arr_poses.poses.push_back(poses_.at(i));
  }
  pub_path_.publish(arr_poses);

  // =======================================================

  nav_msgs::Path arr_poses_gt;
  arr_poses_gt.header.stamp = pose_gt.header.stamp;
  arr_poses_gt.header.seq = poses_count_;
  arr_poses_gt.header.frame_id = "global";
  for (size_t i = 0; i < poses_gt_.size();
       i += std::floor(poses_gt_.size() / 16384.0) + 1) {
    arr_poses_gt.poses.push_back(poses_gt_.at(i));
  }
  pub_path_gt_.publish(arr_poses_gt);

  // Move forward in time
  poses_count_++;
}