#include <filter/KalmanFilter.h>

#include <Eigen/QR>
#include <cmath>

KalmanFilter::KalmanFilter(ParamsManager* params) {
  params_ = params;

  // Init State vector
  state_t_.setZero();
  state_t_1_.setZero();

  // Set constant velocity
  // TODO(alericcardi): create const variable
  state_t_(3, 0) = params_->init_acceleration[0];
  state_t_(4, 0) = params_->init_acceleration[1];
  state_t_(5, 0) = params_->init_acceleration[2];

  // Init Covariance noise
  P_.setZero();

  // Set Covariance process noise
  D_.setZero();
  // TODO(alericcardi): create const variable
  D_(0, 0) = std::pow(params_->noise_process[0], 2);
  D_(1, 1) = std::pow(params_->noise_process[1], 2);
  D_(2, 2) = std::pow(params_->noise_process[2], 2);

  // Set Covariance GPS observation noise
  R_gps_.setZero();
  R_gps_(0, 0) = std::pow(params_->noise_gps[0], 2);
  R_gps_(1, 1) = std::pow(params_->noise_gps[1], 2);
  R_gps_(2, 2) = std::pow(params_->noise_gps[2], 2);

  // Set Covariance Radar observation noise
  R_radar_.setZero();
  R_radar_(0, 0) = std::pow(params_->noise_radar[0], 2);
  R_radar_(1, 1) = std::pow(params_->noise_radar[1], 2);
  R_radar_(2, 2) = std::pow(params_->noise_radar[2], 2);
}

void KalmanFilter::initialization_step(const GPS_DATA& gps1) {
  // Save the current timestamp
  cur_time_ = gps1.timestamp;

  // Set the state as the initial GPS position
  state_t_.block<3, 1>(0, 0) << gps1.pose;
}

void KalmanFilter::propagation_step(double timestamp) {
  // ---------------------------------------------------------------------------
  // Compute the Velocity

  if (dt_ > 0.1) {
    state_t_.tail(3) =
        (state_t_.head(3) - state_t_1_.head(3)) / (dt_ + 0.00001);
    state_t_1_ = state_t_;
  }

  // ---------------------------------------------------------------------------
  // Update the time offset (dt)

  dt_ = timestamp - cur_time_;
  cur_time_ = timestamp;

  // ---------------------------------------------------------------------------
  // Update the State-transition matrix
  update_F();

  // ---------------------------------------------------------------------------
  // Propagate the state and coovariance

  state_t_ = F_ * state_t_;
  P_ = F_ * P_ * F_.transpose() + D_;
}

void KalmanFilter::correction_gps(GPS_DATA& gps_m) {
  Eigen::Matrix<double, 6, 6> H_t = compute_H_gps();
  Eigen::Matrix<double, 6, 1> Y_t = Eigen::Matrix<double, 6, 1>::Zero();

  // Compute the residual between GPS and State
  Y_t.topLeftCorner(3, 1) = gps_m.pose - state_t_.head(3);

  correction_step(Y_t, H_t, R_gps_);
}

void KalmanFilter::correction_radar(RADAR_DATA& radar_m) {
  Eigen::Matrix<double, 6, 6> H_t = compute_H_radar();
  Eigen::Matrix<double, 6, 1> Y_t = Eigen::Matrix<double, 6, 1>::Zero();

  // Compute the residual between Radar and State
  Y_t.head(3) = radar_m.beam - h_radar();

  correction_step(Y_t, H_t, R_radar_);
}

Eigen::Vector3d KalmanFilter::get_state() { return state_t_.head(3); }

void KalmanFilter::correction_step(Eigen::Matrix<double, 6, 1> Y_t,
                                   Eigen::Matrix<double, 6, 6> H_t,
                                   Eigen::Matrix<double, 6, 6> R_t) {
  Eigen::Matrix<double, 6, 6> S_t;
  Eigen::Matrix<double, 6, 6> S_t_inv;
  Eigen::Matrix<double, 6, 6> K_t;

  // ---------------------------------------------------------------------------
  S_t = H_t * P_ * H_t.transpose() + R_t;
  S_t_inv = S_t.completeOrthogonalDecomposition().pseudoInverse();
  K_t = P_ * H_t.transpose() * S_t_inv;

  // ---------------------------------------------------------------------------
  // Correct state and coovariance
  state_t_ = state_t_ + K_t * Y_t;
  P_ = P_ - K_t * S_t * K_t.transpose();
}

void KalmanFilter::update_F() {
  // Init of the State-transistion matrix
  F_.block<1, 6>(0, 0) << 1, 0, 0, dt_, 0, 0;
  F_.block<1, 6>(1, 0) << 0, 1, 0, 0, dt_, 0;
  F_.block<1, 6>(2, 0) << 0, 0, 1, 0, 0, dt_;
  F_.block<1, 6>(3, 0) << 0, 0, 0, 1, 0, 0;
  F_.block<1, 6>(4, 0) << 0, 0, 0, 0, 1, 0;
  F_.block<1, 6>(5, 0) << 0, 0, 0, 0, 0, 1;
}

Eigen::Matrix<double, 6, 6> KalmanFilter::compute_H_gps() {
  Eigen::Matrix<double, 6, 6> H_t = Eigen::Matrix<double, 6, 6>::Zero();

  H_t.block<3, 3>(0, 0) << Eigen::Matrix3d::Identity();

  return H_t;
}

Eigen::Matrix<double, 6, 6> KalmanFilter::compute_H_radar() {
  Eigen::Matrix<double, 6, 6> H_t = Eigen::Matrix<double, 6, 6>::Zero();
  double x = state_t_(0, 0) - params_->init_pose_radar(0, 0);
  double y = state_t_(1, 0) - params_->init_pose_radar(1, 0);
  double z = state_t_(2, 0) - params_->init_pose_radar(2, 0);
  double x_2 = std::pow(x, 2);
  double y_2 = std::pow(y, 2);
  double z_2 = std::pow(z, 2);
  double ro = std::pow(x_2 + y_2 + z_2, 0.5);
  double ro_2 = std::pow(ro, 2);
  double sqrt_x_2_y_2 = std::pow(x_2 + y_2, 0.5);

  H_t.block<1, 3>(0, 0) << x / ro, y / ro, z / ro;
  H_t.block<1, 3>(1, 0) << (x * z) / (ro_2 * sqrt_x_2_y_2),
      (y * z) / (ro_2 * sqrt_x_2_y_2), (-sqrt_x_2_y_2 / ro_2);
  H_t.block<1, 3>(2, 0) << (-y / (x_2 + y_2)), (x / (x_2 + y_2)), 0;

  return H_t;
}

Eigen::Vector3d KalmanFilter::h_radar() {
  Eigen::Vector3d z_radar;
  // From global to local coordinates
  double x = state_t_(0, 0) - params_->init_pose_radar(0, 0);
  double y = state_t_(1, 0) - params_->init_pose_radar(1, 0);
  double z = state_t_(2, 0) - params_->init_pose_radar(2, 0);

  // From local cartesian coordinates to local polar coordinates
  z_radar(0, 0) =
      std::pow(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2), 0.5);
  z_radar(1, 0) = std::acos(z / z_radar(0, 0));
  z_radar(2, 0) = std::atan(y / (x + 0.0001));

  return z_radar;
}
