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
  
  // convert the predicted state vector to the 3D format

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float norm = sqrt((px * px) + (py * py));
  
  // create vector to represent h(x) ((or z_pred)) function for RADAR updating
  VectorXd z_pred(3);
  
  // convert cartesian predictions into polar coordinates
  if (norm > 0.00001)
  {
	z_pred(0) = norm;
	z_pred(1) = atan2(py,px);
	z_pred(2) = ((px * vx) + (py * vy)) / norm;
  }
  
  else
  {
	  z_pred = (H_ * x_);
  }
  
  
  // compare measurements with predictions
  VectorXd y = z - z_pred;
 
  // Normalize phi angle
  while (y(1) > M_PI) y(1) -= 2.*M_PI;
  while (y(1) < -M_PI) y(1) += 2.*M_PI;
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
  
}
