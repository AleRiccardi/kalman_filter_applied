#include <core/KalmanFilter.h>

#include <Eigen/QR>
#include <cmath>

KalmanFilter::KalmanFilter(ParamsManager* params) {
  params_ = params;

  // Init State w/ const acc.
  state_.setZero();
  state_.block<3, 1>(3, 0) = Eigen::MatrixXd::Ones(3, 1) * 0.02;

  // Init the Covariance matrix
  P_.setZero();

  // Init the Observation matrix
  H_.setIdentity();

  // Init Propagation covariance noise
  D_.setZero();
  D_.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * std::pow(0.1, 2);
  std::cout << "D" << std::endl;
  std::cout << D_ << std::endl;

  // Init Propagation covariance noise
  R_.setZero();
  R_(0, 0) = params->gps_noise[0];
  R_(1, 1) = params->gps_noise[1];
  R_(2, 2) = params->gps_noise[2];
  std::cout << "R" << std::endl;
  std::cout << R_ << std::endl;
}

void KalmanFilter::Initialize(const GPSDATA& gps1, const GPSDATA& gps2) {
  // Current timestamp
  timestamp_ = gps1.timestamp;
  // Time offset between two GPS measurements
  dt_ = gps2.timestamp - gps1.timestamp;
  // Set the state as the initial GPS position
  state_.block<3, 1>(0, 0) << gps1.pose;
}

void KalmanFilter::Propagation(double timestamp) {
  // Update F with the correct time offset (dt)
  UpdateF(timestamp);

  // Propagate the state

  // Propagate the covariance
}

void KalmanFilter::Correction(Eigen::Matrix<double, 3, 1> gps_pose) {
  Eigen::Matrix<double, 9, 1> residual;
  Eigen::Matrix<double, 9, 9> S;
  Eigen::Matrix<double, 9, 9> K;
  residual.setZero();

  // Compute the residual and the Kalman Gain

  // Correct the state

  // Correct the covariance
}

void KalmanFilter::UpdateF(double timestamp) {
  // Update time offset
  dt_ = timestamp - timestamp_;
  // Update current time
  timestamp_ = timestamp;

  F_.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
  F_.block<3, 3>(3, 3) << Eigen::MatrixXd::Identity(3, 3);
  F_.bottomRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
  F_.block<3, 3>(0, 3) << Eigen::MatrixXd::Identity(3, 3) * dt_;
  F_.block<3, 3>(3, 6) << Eigen::MatrixXd::Identity(3, 3) * dt_;
  F_.block<3, 3>(0, 6) << Eigen::MatrixXd::Identity(3, 3) *
                              (0.5 * std::pow(dt_, 2));
}
void KalmanFilter::UpdateH() {}

Eigen::Matrix<double, 3, 1> KalmanFilter::GetState() {
  return state_.topLeftCorner(3, 1);
}