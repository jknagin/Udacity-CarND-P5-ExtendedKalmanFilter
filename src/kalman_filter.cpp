#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  Eigen::MatrixXd K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
  x_ += K * (z - H_ * x_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  // Use linearized measurement model to update state estimate covariance
  Eigen::MatrixXd Hj = Tools().CalculateJacobian(x_);

  Eigen::MatrixXd K = P_ * Hj.transpose() * (Hj * P_ * Hj.transpose() + R_).inverse();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * Hj) * P_;

  // Use actual (nonlinear) measurement model to update state estimate with Kalman gain
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = std::sqrt(px * px + py * py);
  double theta = std::atan2(py, px);
  double rhoDot = (px * vx + py * vy) / rho;
  Eigen::Matrix<double, 3, 1> zBar;
  zBar << rho, theta, rhoDot; // Expected measurement given state
  Eigen::Matrix<double, 3, 1> innovation = z - zBar;

  while (innovation(1) > M_PI)
  {
    innovation(1) -= 2 * M_PI;
  }
  while (innovation(1) < -M_PI)
  {
    innovation(1) += 2 * M_PI;
  }

  x_ += K * innovation;
}
