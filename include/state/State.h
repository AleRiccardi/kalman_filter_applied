#ifndef KFA_STATE_H
#define KFA_STATE_H

#include <unordered_map>
#include <vector>

#include "types/Pose.h"
#include "types/Vec.h"

/**
 * @brief State of our filter
 *
 * This state has all the current estimates for the filter.
 * This system is modeled after the MSCKF filter, thus we have a sliding window
 * of clones. We additionally have more parameters for online estimation of
 * calibration and SLAM features. We also have the covariance of the system,
 * which should be managed using the StateHelper class.
 */
class State {
 public:
  /**
   * @brief Default Constructor (will initialize variables to defaults)
   */
  State();

  ~State() {}

  /**
   * @brief Calculates the current max size of the covariance
   * @return Size of the current covariance matrix
   */
  int max_covariance_size() { return (int)Cov_.rows(); }

  /// Current timestamp (should be the last update time!)
  double timestamp_;

  Pose pose_;

 private:

  /// Covariance of all active variables
  Eigen::MatrixXd Cov_;

  /// Vector of variables
  std::vector<Type*> variables_;
};

#endif  // KFA_STATE_H