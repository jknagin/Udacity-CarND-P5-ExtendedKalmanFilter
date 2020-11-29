#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth)
{
   VectorXd rmse(4);
   rmse << 0, 0, 0, 0;
   Eigen::VectorXd residual(4);

   // TODO: YOUR CODE HERE
   // check the validity of the following inputs:
   //  * the estimation vector size should not be zero
   //  * the estimation vector size should equal ground truth vector size

   if (estimations.empty())
      return rmse;
   if (estimations.size() != ground_truth.size())
      return rmse;

   // TODO: accumulate squared residuals
   for (int i = 0; i < estimations.size(); ++i)
   {
      residual = estimations[i] - ground_truth[i];
      for (int j = 0; j < residual.size(); ++j)
      {
         residual[j] *= residual[j];
      }
      rmse += residual;
   }

   // TODO: calculate the mean
   rmse /= estimations.size();

   // TODO: calculate the squared root
   for (int i = 0; i < rmse.size(); ++i)
   {
      rmse[i] = std::sqrt(rmse[i]);
   }
   // return the result
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{
   /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Hj(3, 4);
   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // TODO: YOUR CODE HERE

   // check division by zero
   if (px == 0 && py == 0)
      return Hj;
   // compute the Jacobian matrix
   Hj = Eigen::Matrix<double, 3, 4>::Zero();
   Hj(0, 0) = px / std::sqrt(px * px + py * py);
   Hj(0, 1) = py / std::sqrt(px * px + py * py);
   Hj(1, 0) = -py / (px * px + py * py);
   Hj(1, 1) = px / (px * px + py * py);
   Hj(2, 0) = py * (vx * py - vy * px) / std::pow(px * px + py * py, 1.5);
   Hj(2, 1) = px * (py * vx - px * vy) / std::pow(px * px + py * py, 1.5);
   Hj(2, 2) = px / std::sqrt(px * px + py * py);
   Hj(2, 3) = py / std::sqrt(px * px + py * py);
   return Hj;
}
