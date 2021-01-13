#include <Eigen/Eigen>

#include "core/KalmanFilter.h"
#include "utils/ParamsManager.h"

/**
 * @brief GPS data struct.
 *
 */
struct GPSDATA {
  /// Timestamp of the reading
  double timestamp;

  /// 3D Pose
  Eigen::Matrix<double, 3, 1> pose;
};

class CoreManager {
 public:
  /**
   * @brief Construct a new Core Manager object
   *
   * @param params of the system stored in the launch file.
   */
  CoreManager(ParamsManager* params);

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
  /* data */
  ParamsManager* params_;

  KalmanFilter* kf_;

  // Our history of GPS messages (time, pose)
  std::vector<GPSDATA> gps_data_;
};
