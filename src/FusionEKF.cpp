#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Measurement matrix for laser (linear case)
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Initialize the state covariance matrix P (assuming high uncertainty in velocity)
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  // Initialize the state transition matrix F
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Process noise
  noise_ax = 9;
  noise_ay = 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(3); // State vector [px, py, vx, vy]

      // Convert radar from polar to cartesian coordinates
    VectorXd yG = measurement_pack.raw_measurements_(0);
    VectorXd yA = measurement_pack.raw_measurements_(1);
    double v = measurement_pack.raw_measurements_(2);
    

    // Initialize state
    ekf_.x_ << 0, 0, 1;


    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  // Compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update the state transition matrix F with the new elapsed time
  // ekf_.F_(0, 2) = dt;
  // ekf_.F_(1, 3) = dt;

  // // Update the process noise covariance matrix Q
  // double dt_2 = dt * dt;
  // double dt_3 = dt_2 * dt;
  // double dt_4 = dt_3 * dt;

  // ekf_.Q_ = MatrixXd(4, 4);
  // ekf_.Q_ <<  dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
  //             0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
  //             dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
  //             0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

  // Perform the prediction step
  ekf_.Predict(VectorXd yG, VectorXd yA, double dt);

  /**
   * Update
   */

  // if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
  //   // Radar updates (non-linear)
  //   // Compute Jacobian matrix Hj
  //   Hj_ = tools.CalculateJacobian(ekf_.x_);
  //   ekf_.H_ = Hj_; // Use Jacobian matrix for radar update
  //   ekf_.R_ = R_radar_;
  ekf_.UpdateEKFStep1(measurement_pack.raw_measurements_); // Extended Kalman Filter update
  ekf_.UpdateEKFStep2();
  // } else {
  //   // Laser updates (linear)
  //   ekf_.H_ = H_laser_;
  //   ekf_.R_ = R_laser_;
  //   ekf_.Update(measurement_pack.raw_measurements_); // Standard Kalman Filter update
  // }

  // Print the updated state and covariance matrix
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
