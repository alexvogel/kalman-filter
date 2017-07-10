#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}
#define EPS 0.0001 // A very small number
#define EPS2 0.0000001

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;

}

VectorXd Tools::State2radar(const VectorXd& x_state) {

	// calc rho ( range: radial distance of object from vehicle )
	double rho = sqrt( pow(x_state(0), 2) + pow(x_state(1), 2) );

	// calc phi ( bearing: angle between x-axis and rho of object)
	double phi = atan( x_state(1) / x_state(0) );
	cout << "phi: " << phi << endl;

	// calc rhod-dot (radial velocity: change of rho)
	double rho_dot = (x_state(0) * x_state(2) + x_state(1) * x_state(3)) / rho;

	VectorXd x_radar(3);
	x_radar << rho, phi, rho_dot;

	return x_radar;
}

VectorXd Tools::Radar2state(const VectorXd& x_radar) {

	/**
	 * only the px and py will be determined
	 * vx and vy are set to zero
	 */

	VectorXd x_state(4);
	x_state << 0,0,0,0;

	x_state(0) = cos(x_radar(1)) * x_radar(0);
	x_state(1) = sin(x_radar(1)) * x_radar(0);

	return x_state;
}

MatrixXd Tools::CalculateJacobianAvoge(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

    float px_square_plus_py_square = pow(px, 2) + pow(py, 2);

	//check division by zero
    if(px_square_plus_py_square < 0.0001)
    {
        std::cout << "CalculateJacobian () - Error - Division By Zero";
        return Hj;
    }

	//compute the Jacobian matrix
    Hj(0,0) = px / sqrt( px_square_plus_py_square );
    Hj(0,1) = py / sqrt( px_square_plus_py_square );
    Hj(0,2) = 0.;
    Hj(0,3) = 0.;
    Hj(1,0) = -1 * ( py / px_square_plus_py_square );
    Hj(1,1) = px / px_square_plus_py_square;
    Hj(1,2) = 0.;
    Hj(1,3) = 0.;
    Hj(2,0) = py * ( vx * py - vy * px) / ( pow(px_square_plus_py_square, 2/3));
    Hj(2,1) = px * ( vy * px - vx * py) / ( pow(px_square_plus_py_square, 2/3));
    Hj(2,2) = px / sqrt( px_square_plus_py_square );
    Hj(2,3) = py / sqrt( px_square_plus_py_square );

	return Hj;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Code from lectures quizes
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  MatrixXd Hj(3,4);
  // Deal with the special case problems
  if (fabs(px) < EPS and fabs(py) < EPS){
	  px = EPS;
	  py = EPS;
  }
  // Pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  // Check division by zero
  if(fabs(c1) < EPS2){
	c1 = EPS2;
  }
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  // Compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  return Hj;
}
