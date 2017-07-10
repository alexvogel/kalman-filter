#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

Tools tools2;

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
  /**
  TODO:
    * predict the state
  */

  // predict the state matrix
  x_ = F_ * x_;

  // predict the process covariance matrix
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

  std::cout << "... prediction finished" << std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;


  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // H_ is already the Hj (jacobian) when UpdateEKF is called

  // convert the predicted state from the 4D world (px, py, vx, vy) to the 3D world of radar (rho, phi, rho-dot)
  VectorXd h = tools2.State2radar(x_); //
  VectorXd y = z - h;

  // if phi (y(1)) is out of range -pi < phi < +pi, then it will be normalized into this range
  while(y(1) < -M_PI)
  {
	y(1) += 2 * M_PI;
	cout << "normalizing phi..." << endl;
  }
  while(y(1) > M_PI)
  {
	y(1) -= 2 * M_PI;
	cout << "normalizing phi..." << endl;
  }
  cout << "y = " << y << endl;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;


}
