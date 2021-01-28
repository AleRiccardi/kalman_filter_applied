#ifndef KFA_TYPES_H
#define KFA_TYPES_H

#include <Eigen/Dense>

/**
 * @brief GPS data struct.
 *
 */
struct GPS_DATA {
  /// Timestamp of the reading
  double timestamp;

  /// 3D Pose
  Eigen::Vector3d pose;
};

struct RADAR_DATA {
  /// Timestamp of the reading
  double timestamp;

  /// Beam coordinates
  Eigen::Vector3d beam;
};

#endif  // KFA_TYPES_H
