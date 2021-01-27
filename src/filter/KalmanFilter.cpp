#include <filter/KalmanFilter.h>

#include <Eigen/QR>
#include <cmath>

KalmanFilter::KalmanFilter(ParamsManager* params) {
  params_ = params;

  // Init State w/ const acc.
  state_.setZero();
  state_.block<3, 1>(3, 0) = Eigen::MatrixXd::Ones(3, 1) * 0.02;

  // Init the Covariance matrix
  P_.setZero();

  // Init propagation covariance noise
  D_.setZero();
  D_.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * 0.1;
}

void KalmanFilter::initialize(const GPS_DATA& gps1, const GPS_DATA& gps2) {
  // Current timestamp
  timestamp_ = gps1.timestamp;
  // Time offset between two GPS measurements
  dt_ = gps2.timestamp - gps1.timestamp;
  // Set the state as the initial GPS position
  state_.block<3, 1>(0, 0) << gps1.pose;
}

void KalmanFilter::propagation(double timestamp) {
  // Update F with the correct time offset (dt)
  update_F(timestamp);

  // Propagate the state
  state_ = F_ * state_;
  // Propagate the covariance
  P_ = F_ * P_ * F_.transpose() + D_;
}

void KalmanFilter::correction_gps(GPS_DATA& gps_m) {
  Eigen::Vector3d gps_p = gps_m.pose;
  Eigen::Matrix<double, 9, 1> residual;
  Eigen::Matrix<double, 9, 9> S;
  Eigen::Matrix<double, 9, 9> K;

  update_H_gps();

  // Compute residual between GPS measurement and State
  residual.setZero();
  residual.topLeftCorner(3, 1) = gps_p - state_.topLeftCorner(3, 1);

  // ...
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.completeOrthogonalDecomposition().pseudoInverse();

  state_ = state_ + K * residual;
  P_ = (Eigen::MatrixXd::Identity(9, 9) * K * H_) * P_;

  // Print the difference between the state and the gps
  // std::cout << "GPS" << std::endl;
  // std::cout << gps_p << std::endl;
  // std::cout <<
  // "---------------------------------------------------------------"
  //              "-------------------------------"
  //           << std::endl;
  // std::cout << "STATE" << std::endl;
  // std::cout << state_.topLeftCorner(3, 1) << std::endl;
  // std::cout <<
  // "---------------------------------------------------------------"
  //              "-------------------------------"
  //           << std::endl;
  // std::cout << "RESIDUAL " << std::endl;
  // std::cout << residual << std::endl;
  // std::cout <<
  // "==============================================================="
  //              "==============================="
  //           << std::endl
  //           << std::endl;
}

void KalmanFilter::correction_radar(RADAR_DATA& radar_m) {
  Eigen::Vector3d radar_b = radar_m.beam;
  Eigen::Vector3d h_x = h_radar();
  Eigen::Matrix<double, 9, 1> residual;
  Eigen::Matrix<double, 9, 9> S;
  Eigen::Matrix<double, 9, 9> K;

  update_H_radar();

  // Compute residual between GPS measurement and State
  residual.setZero();
  residual.topLeftCorner(3, 1) = radar_b - h_x;

  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.completeOrthogonalDecomposition().pseudoInverse();

  state_ = state_ + K * residual;
  P_ = P_ - K * S * K.transpose();
  // P_ = (Eigen::MatrixXd::Identity(9, 9) * K * H_) * P_;

  // Print the difference between the state and the gps
  // std::cout <<
  // "---------------------------------------------------------------"
  //           << std::endl;
  // std::cout << "RADAR " << std::endl;
  // std::cout << radar_b << std::endl;
  // std::cout <<
  // "---------------------------------------------------------------"
  //           << std::endl;
  // std::cout << "h-func " << std::endl;
  // std::cout << h_x << std::endl;
  // std::cout <<
  // "---------------------------------------------------------------"
  //           << std::endl;
  // std::cout << "RESIDUAL " << std::endl;
  // std::cout << residual << std::endl;
  // std::cout <<
  // "---------------------------------------------------------------"
  //           << std::endl;
  // std::cout << "K * residual " << std::endl;
  // std::cout << K * residual << std::endl;
  // std::cout <<
  // "==============================================================="
  //              "==============================="
  //           << std::endl
  //           << std::endl;
}

Eigen::Vector3d KalmanFilter::GetState() { return state_.topLeftCorner(3, 1); }

void KalmanFilter::update_F(double timestamp) {
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

void KalmanFilter::update_H_gps() {
  // For the GPS, H is an identity matrix
  H_.setIdentity();

  // Update GPS propagation covariance noise
  R_.setZero();
  R_(0, 0) = params_->noise_gps[0];
  R_(1, 1) = params_->noise_gps[1];
  R_(2, 2) = params_->noise_gps[2];
}

void KalmanFilter::update_H_radar() {
  double x = state_(0, 0) - params_->init_pose_radar(0, 0);
  double y = state_(1, 0) - params_->init_pose_radar(1, 0);
  double z = state_(2, 0) - params_->init_pose_radar(2, 0);
  double x_2 = std::pow(x, 2);
  double y_2 = std::pow(y, 2);
  double z_2 = std::pow(z, 2);
  double ro = std::pow(x_2 + y_2 + z_2, 0.5);
  double ro_2 = std::pow(ro, 2);
  double sqrt_x_2_y_2 = std::pow(x_2 + y_2, 0.5);

  H_.setIdentity();

  H_.block<1, 3>(0, 0) << x / ro, y / ro, z / ro;
  H_.block<1, 3>(1, 0) << (x * z) / (ro_2 * sqrt_x_2_y_2),
      (y * z) / (ro_2 * sqrt_x_2_y_2), (-sqrt_x_2_y_2 / ro_2);
  H_.block<1, 3>(2, 0) << (-y / (x_2 + y_2)), (x / (x_2 + y_2)), 0;

  // Update RADAR propagation covariance noise
  R_.setZero();
  R_(0, 0) = params_->noise_radar[0];
  R_(1, 1) = params_->noise_radar[1];
  R_(2, 2) = params_->noise_radar[2];
}

Eigen::Vector3d KalmanFilter::h_radar() {
  Eigen::Vector3d z_radar;

  // From global to local coordinates
  double x = state_(0, 0) - params_->init_pose_radar(0, 0);
  double y = state_(1, 0) - params_->init_pose_radar(1, 0);
  double z = state_(2, 0) - params_->init_pose_radar(2, 0);

  // From local cartesian coordinates to local polar coordinates
  z_radar(0, 0) =
      std::pow(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2), 0.5);
  z_radar(1, 0) = std::acos(z / z_radar(0, 0));
  z_radar(2, 0) = std::atan(y / (x + 0.0001));

  return z_radar;
}
