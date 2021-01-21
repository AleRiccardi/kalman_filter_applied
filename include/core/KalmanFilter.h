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
  void initialize(const GPSDATA& gps1, const GPSDATA& gps2);

  /**
   * @brief Propagation step.
   *
   */
  void propagation(double timestamp);

  /**
   * @brief Correction step.
   *
   */
  void correction(Eigen::Matrix<double, 3, 1> gps_pose);

  Eigen::Matrix<double, 3, 1> GetState();

 private:
  void update_F(double timestamp);

  void update_H();

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
