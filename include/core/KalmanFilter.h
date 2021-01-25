#ifndef KF_APPLIED_KALMAN_FILTER_H
#define KF_APPLIED_KALMAN_FILTER_H

#include <Eigen/Dense>

#include "utils/ParamsManager.h"
#include "utils/types.h"

/**
 * @brief The Kalman Filter class.
 *
 */
class KalmanFilter {
 public:
  /**
   * @brief Construct a new Kalman Filter object.
   *
   * @param params of the system stored in the launch file.
   */
  KalmanFilter(ParamsManager* params);

  /**
   * @brief Destroy the Kalman Filter object.
   *
   */
  ~KalmanFilter() = default;

  /**
   * @brief Initialize the state and timestamp.
   */
  void initialize(const GPS_DATA& gps1, const GPS_DATA& gps2);

  /**
   * @brief Propagation step.
   *
   */
  void propagation(double timestamp);

  /**
   * @brief GPS correction step.
   *
   */
  void correction_gps(GPS_DATA& gps_m);

  /**
   * @brief RADAR correction step.
   *
   */
  void correction_radar(RADAR_DATA& radar_m);

  Eigen::Vector3d GetState();

 private:
  void update_F(double timestamp);

  void update_H_gps();
  void update_H_radar();
  Eigen::Vector3d h_radar();

  // Parameters system manager
  ParamsManager* params_;

  // State vector
  Eigen::Matrix<double, 9, 1> state_;
  // Covariance matrix
  Eigen::Matrix<double, 9, 9> P_;
  // State-Transition matrix
  Eigen::Matrix<double, 9, 9> F_;
  // Observation Matrix
  Eigen::Matrix<double, 9, 9> H_;
  // propagation covariance noise
  Eigen::Matrix<double, 9, 9> D_;
  // Observation covariance noise
  Eigen::Matrix<double, 9, 9> R_;

  double dt_ = 0;
  double timestamp_ = 0;
};

#endif  // KF_APPLIED_KALMAN_FILTER_H