#ifndef KF_APPLIED_CORE_MANAGER_H
#define KF_APPLIED_CORE_MANAGER_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>

#include "core/KalmanFilter.h"
#include "utils/ParamsManager.h"
#include "utils/types.h"

class CoreManager {
 public:
  /**
   * @brief Construct a new Core Manager object
   *
   * @param params of the system stored in the launch file.
   */
  CoreManager(ros::NodeHandle nh, ParamsManager* params);

  /**
   * @brief Destroy the Core Manager object
   *
   */
  ~CoreManager() = default;

  /**
   * @brief Store Ground Truth measurement.
   *
   * @param timestamp Timestamp of gt measurement.
   * @param pose 3D pose.
   */
  void feed_m_gt(double timestamp, Eigen::Vector3d pose);

  /**
   * @brief Store incoming GPS measurement.
   *
   * @param timestamp Timestamp of gps reading
   * @param pose 3D pose.
   */
  void feed_m_gps(double timestamp, Eigen::Vector3d pose);

  /**
   * @brief Store Radar measurement.
   *
   * @param timestamp Timestamp of radar measurement.
   * @param beam of the radar.
   */
  void feed_m_radar(double timestamp, Eigen::Vector3d beam);

  /**
   * @brief Estimate the state given the collected measurements.
   *
   */
  void state_estimation();

  /**
   * @brief Display the current state.
   *
   */
  void display();

 private:
  bool initialize();
  bool pop_gps(std::vector<GPS_DATA>& gps_v);
  bool pop_radar(std::vector<RADAR_DATA>& radar_v);

  ParamsManager* params_;
  KalmanFilter* kf_;

  // Our history of GPS messages (time, pose)
  std::vector<GPS_DATA> gps_data_;
  // Our history of GT messages (time, pose)
  std::vector<GPS_DATA> gt_data_;
  // Our history of RADAR messages
  std::vector<RADAR_DATA> radar_data_;

  double curr_time_;
  bool is_initialized_ = false;
  int count_poses_ = 0;

  ros::Publisher pub_estimation_, pub_estimation_path_, pub_gt_pose_,
      pub_gt_path;
  std::vector<geometry_msgs::PoseStamped> poses_;
  std::vector<geometry_msgs::PoseStamped> poses_gt_;
};

#endif  // KF_APPLIED_CORE_MANAGER_H