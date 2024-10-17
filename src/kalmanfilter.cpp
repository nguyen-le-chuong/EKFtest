#include "kalman_filter.h"
#include <iostream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // Predict the state
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Linear Kalman Filter update
  VectorXd y = z - H_ * x_;  // Measurement residual
  MatrixXd S = H_ * P_ * H_.transpose() + R_;  // Residual covariance
  MatrixXd K = P_ * H_.transpose() * S.inverse();  // Kalman gain

  // Update state
  x_ = x_ + K * y;
  
  // Update covariance matrix
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Extended Kalman Filter update (for non-linear measurements)

  // Convert the predicted state (x_) from Cartesian to polar coordinates
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px);  // atan2 handles the case when px = 0
  double rho_dot = (px * vx + py * vy) / std::max(rho, 0.0001);  // Avoid division by zero

  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;

  VectorXd y = z - h;  // Measurement residual

  // Normalize angle within the range [-π, π]
  while (y(1) > M_PI) y(1) -= 2 * M_PI;
  while (y(1) < -M_PI) y(1) += 2 * M_PI;

  // Compute the Jacobian matrix (Hj should be precomputed and set externally in ProcessMeasurement)
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // Update state
  x_ = x_ + K * y;

  // Update covariance matrix
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
