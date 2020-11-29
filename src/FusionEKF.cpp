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
FusionEKF::FusionEKF()
{
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

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /**
   * Initialization
   */
  if (!is_initialized_)
  {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    Eigen::VectorXd x_in(4);
    Eigen::MatrixXd P_in = Eigen::Matrix<double, 4, 4>::Zero();
    Eigen::MatrixXd F_in = Eigen::Matrix<double, 4, 4>::Identity(); // varying but only in two places (dt)
    Eigen::MatrixXd H_in = Eigen::Matrix<double, 2, 4>::Zero();
    H_in << 1, 0, 0, 0,
        0, 1, 0, 0;
    Eigen::MatrixXd R_in;                                       // varying depending on measurement type
    Eigen::MatrixXd Q_in = Eigen::Matrix<double, 4, 4>::Zero(); // varying depending on dt

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
      double r = measurement_pack.raw_measurements_[0];
      double theta = measurement_pack.raw_measurements_[1];
      x_in << r * std::cos(theta), r * std::sin(theta), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      // TODO: Initialize state.
      x_in << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    P_in << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 999, 0,
        0, 0, 0, 999;

    ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.0e6;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;

  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  double covAX = 9;
  double covAY = 9;
  ekf_.Q_ << dt4 / 4 * covAX, 0, dt3 / 2 * covAX, 0,
      0, dt4 / 4 * covAY, 0, dt3 / 2 * covAY,
      dt3 / 2 * covAX, 0, dt2 * covAX, 0,
      0, dt3 / 2 * covAY, 0, dt2 * covAY;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  Eigen::VectorXd z = measurement_pack.raw_measurements_;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // TODO: Radar updates
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(z);
  }
  else
  {
    // TODO: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
