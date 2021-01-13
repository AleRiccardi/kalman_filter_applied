#include "utils/ParamsManager.h"

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
   * @brief Propagation step.
   *
   */
  void Propagation();

  /**
   * @brief Correction step.
   *
   */
  void Correction();

 private:
  ParamsManager* params_;
};
