#include <core/CoreManager.h>

#include <utility>

CoreManager::CoreManager(ParamsManager* params) {
  params_ = params;
  kf_ = new KalmanFilter(params);
}

void CoreManager::FeedMeasurementGPS(double timestamp, Eigen::Vector3d pose) {
  GPSDATA data;
  data.timestamp = timestamp;
  data.pose = std::move(pose);

  gps_data_.emplace_back(data);
}

void CoreManager::StateEstimation() {}

void CoreManager::Display() {}
