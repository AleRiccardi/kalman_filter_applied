#ifndef KFA_KALMAN_FILTER_H
#define KFA_KALMAN_FILTER_H

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
   *
   * @param gps1
   */
  void initialization_step(const GPS_DATA& gps1);

  /**
   * @brief Propagation step.
   *
   */
  void propagation_step(double timestamp);

  /**
   * @brief GPS correction step.
   *
   * @param gps_m GPS measurement.
   */
  void correction_gps(GPS_DATA& gps_m);

  /**
   * @brief Radar correction step.
   *
   * @param radar_m Radar measurement.
   */
  void correction_radar(RADAR_DATA& radar_m);

  /**
   * @brief Get the current state vector.
   *
   */
  Eigen::Vector3d get_state();

 private:
  /**
   * @brief General correction step.
   *
   * @param Y_t Residual between measurement and observation.
   * @param H_t Hessian matrix of the observation.
   * @param R_t Covariance observation matrix.
   */
  void correction_step(Eigen::Matrix<double, 6, 1> Y_t,
                       Eigen::Matrix<double, 6, 6> H_t,
                       Eigen::Matrix<double, 6, 6> R_t);
  /**
   * @brief Update the state-transistion matrix.
   *
   */
  void update_F();

  /**
   * @brief Compute the Hessian matrix for a GPS measurement.
   *
   * @return Eigen::Matrix<double, 6, 6> The computed GPS Hessian matrix.
   */
  Eigen::Matrix<double, 6, 6> compute_H_gps();

  /**
   * @brief Compute the Hessian matrix for a Radar measurement.
   *
   * @return Eigen::Matrix<double, 6, 6> the computed Radar Hessian matrix.
   */
  Eigen::Matrix<double, 6, 6> compute_H_radar();

  /**
   * @brief Retrieve the radar observation based on the current state.
   *
   * @return Eigen::Vector3d the generated observation.
   */
  Eigen::Vector3d h_radar();

  // Parameters system manager
  ParamsManager* params_;

  // State vector
  Eigen::Matrix<double, 6, 1> state_t_;
  Eigen::Matrix<double, 6, 1> state_t_1_;
  // Covariance matrix
  Eigen::Matrix<double, 6, 6> P_;

  // State-transition matrix
  Eigen::Matrix<double, 6, 6> F_;

  // Covariance process noise
  Eigen::Matrix<double, 6, 6> D_;
  // Covariance gps noise matrix
  Eigen::Matrix<double, 6, 6> R_gps_;
  // Covariance gps noise matrix
  Eigen::Matrix<double, 6, 6> R_radar_;

  // Current estimation time
  double dt_ = 0;
  double cur_time_ = 0;
};

#endif  // KFA_KALMAN_FILTER_H