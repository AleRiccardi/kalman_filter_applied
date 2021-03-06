#ifndef KFA_FILTER_MANAGER_H
#define KFA_FILTER_MANAGER_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>

#include "filter/KalmanFilter.h"
#include "utils/ParamsManager.h"
#include "utils/types.h"

class FilterManager {
 public:
  /**
   * @brief Construct a new Core Manager object
   *
   * @param params of the system stored in the launch file.
   */
  FilterManager(ros::NodeHandle nh, ParamsManager* params);

  /**
   * @brief Destroy the Core Manager object
   *
   */
  ~FilterManager() = default;

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
   * @brief Display by publishing the current state.
   *
   */
  void display();

 private:
  bool initialize();
  bool pop_gps(std::vector<GPS_DATA>& gps_v);
  bool pop_radar(std::vector<RADAR_DATA>& radar_v);

  ParamsManager* params_;
  KalmanFilter* kf_;

  double cur_time_;
  int count_poses_ = 0;
  bool is_initialized_ = false;

  // Our history of GPS messages (time, pose)
  std::vector<GPS_DATA> gps_data_;
  // Our history of GT messages (time, pose)
  std::vector<GPS_DATA> gt_data_;
  // Our history of Radar messages
  std::vector<RADAR_DATA> radar_data_;

  ros::Publisher pub_estimation_;
  ros::Publisher pub_estimation_path_;
  ros::Publisher pub_gt_pose_;
  ros::Publisher pub_gt_path;
  std::vector<geometry_msgs::PoseStamped> poses_;
  std::vector<geometry_msgs::PoseStamped> poses_gt_;
};

#endif  // KFA_FILTER_MANAGER_H