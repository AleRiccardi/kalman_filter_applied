#ifndef KF_APPLIED_TYPES_H
#define KF_APPLIED_TYPES_H

#include <Eigen/Dense>

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

#endif  // KF_APPLIED_TYPES_H
