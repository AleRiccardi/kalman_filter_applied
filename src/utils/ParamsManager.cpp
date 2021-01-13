#include "utils/ParamsManager.h"

#include <ros/ros.h>

ParamsManager::ParamsManager(ros::NodeHandle& nh) {
  // ===========================================================================
  // GPS Noise Matrix
  // ===========================================================================
  std::vector<double> gps_noise_v = {gps_noise(0), gps_noise(1), gps_noise(2)};
  nh.param<std::vector<double>>("gps_noise", gps_noise_v, gps_noise_v);
  assert(gps_noise_v.size() == 3);
  gps_noise << gps_noise_v[0], gps_noise_v[1], gps_noise_v[2];

  PrintNoise();
}
