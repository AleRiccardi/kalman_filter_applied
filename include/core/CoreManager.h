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
   * @brief Store incoming GPS measurement.
   *
   * @param timestamp Timestamp of gps reading
   * @param pose 3D pose.
   */
  void feed_m_gps(double timestamp, Eigen::Vector3d pose);

  /**
   * @brief Store Ground Truth measurement.
   *
   * @param timestamp Timestamp of gt measurement.
   * @param pose 3D pose.
   */
  void feed_m_gt(double timestamp, Eigen::Vector3d pose);

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
  bool pop_measurement(GPSDATA& gps);

  ParamsManager* params_;
  KalmanFilter* kf_;

  // Our history of GPS messages (time, pose)
  std::vector<GPSDATA> gps_data_;
  // Our history of GT messages (time, pose)
  std::vector<GPSDATA> gt_data_;

  bool is_initialized_ = false;

  int poses_count_ = 0;
  ros::Publisher pub_estimation_, pub_estimation_path_, pub_gt_pose_,
      pub_gt_path;
  std::vector<geometry_msgs::PoseStamped> poses_;
  std::vector<geometry_msgs::PoseStamped> poses_gt_;
};

#endif  // KF_APPLIED_CORE_MANAGER_H