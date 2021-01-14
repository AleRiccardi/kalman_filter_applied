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
   * @brief Store incoming gps measurements.
   *
   * @param timestamp Timestamp of imu reading
   * @param pose 3D pose.
   */
  void FeedMeasurementGPS(double timestamp, Eigen::Vector3d pose);

  /**
   * @brief Store Ground Truth pose.
   *
   * @param timestamp Timestamp of imu reading
   * @param pose 3D pose.
   */
  void FeedGT(double timestamp, Eigen::Vector3d pose);

  /**
   * @brief Estimate the state given the collected measurements.
   *
   */
  void StateEstimation();

  /**
   * @brief Display the current state.
   *
   */
  void Display();

 private:
  bool Initialize();
  bool PopMeasurement(GPSDATA& gps);

  ParamsManager* params_;
  KalmanFilter* kf_;

  // Our history of GPS messages (time, pose)
  std::vector<GPSDATA> gps_data_;
  // Our history of GT messages (time, pose)
  std::vector<GPSDATA> gt_data_;

  bool is_initialized_ = false;

  int poses_count_ = 0;
  ros::Publisher pub_pose_, pub_path_, pub_pose_gt_, pub_path_gt_;
  std::vector<geometry_msgs::PoseStamped> poses_;
  std::vector<geometry_msgs::PoseStamped> poses_gt_;
};
