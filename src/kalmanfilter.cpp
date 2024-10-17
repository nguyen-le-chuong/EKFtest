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

void KalmanFilter::Predict(GyroMeasurement gyro, double dt) {
  // Predict the state
  double wx = gyro.wx;
  double wy = gyro.wy;
  double wz = gyro.wz;

  double R31 = x_(0)
  double R32 = x_(1)
  double R33 = x_(2)

  Matrix3d omega_skew;
  omega_skew <<  0,   -wz,   wy,
                  wz,   0,   -wx,
                -wy,  wx,    0;
  x_skew << 0,    -R33,   R32,
            R33,  0,      -R31,
            -R32, R31,    0;  
  Matrix3d F = Matrix3d::Identity() + omega_skew * dt;  // F matrix

  // Predict the new state
  x_ = F * x_;

  sigma_G = GYRO_STD * Matrix3d::Identity()
  Matrix3d Q_ = dt * dt * x_skew * sigma_G * x_skew

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

void KalmanFilter::UpdateEKFStep1(AccelMeasurement accel, GyroMeasurement gyro, double v_t) {
  // Extended Kalman Filter update (for non-linear measurements)


  double R31 = x_(0);
  double R32 = x_(1);
  double R33 = x_(2);

  double ay = accel.ay;          
  double az = accel.az;  

  double wx = gyro.wx;   
  double wy = gyro.wy;   
  double wz = gyro.wz;    


  double ay_centripetal = v_t * wz;       
  double az_centripetal = -v_t * wx;      

  double ay_corrected = ay - ay_centripetal;
  double az_corrected = az - az_centripetal;

  MatrixXd H1(2, 3);
  H1 << 0, 1, 0,
        0, 0, 1;

  Vector2d z1;
  z1 << ay_corrected, az_corrected;

  VectorXd y = z1 - H1 * state;

  MatrixXd S1 = H1 * P_ * H1.transpose() + R1;
  MatrixXd K1 = P_ * H1.transpose() * S1.inverse();

  x_ = x_ + K1 * y;

  MatrixXd I = MatrixXd::Identity(P_.rows(), P_.cols());
  P_ = (I - K1 * H1) * P_;

  // Convert the predicted state (x_) from Cartesian to polar coordinates

}


void KalmanFilter::UpdateEKFStep2() {
  // Extended Kalman Filter update (for non-linear measurements)


      double R31 = x_(0);
      double R32 = x_(1);
      double R33 = x_(2);

      MatrixXd H2(1, 3);
      H2 << 0, 1, 0,
            0, 0, 1;

      double z2 = sqrt(1 - R32 * R32 - R33 * R33)

      double y = z2 - H2 * state;

      MatrixXd S2 = H2 * P_ * H2.transpose() + R2;
      MatrixXd K2 = P_ * H2.transpose() * S2.inverse();

      x_ = x_ + K2 * y;

      MatrixXd I = MatrixXd::Identity(P_.rows(), P_.cols());
      P_ = (I - K2 * H2) * P_;

  // Convert the predicted state (x_) from Cartesian to polar coordinates

}
