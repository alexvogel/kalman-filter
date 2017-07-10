#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools tools;

/*
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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  H_laser_ << 1., 0., 0., 0.,
		      0., 1., 0., 0.;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
	cout << "initialization ..." << endl;
	// create an initial state covariance matrix
	ekf_.P_ = MatrixXd(4,4);
	ekf_.P_ << 1., 0, 0, 0,
			   0, 1., 0, 0,
			   0, 0, 100., 0,
			   0, 0, 0, 100.;

    // create state matrix after first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

//    cout << "measurement_pack.raw_measurements_ " << measurement_pack.raw_measurements_ << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "convert radar from polar to cartesian..." << endl;
      ekf_.x_ = tools.Radar2state(measurement_pack.raw_measurements_);

      cout << "x_: " << ekf_.x_ << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      laser measurement only has positions (px, py) - no velocities (vx, vy)
      initializing vx and vy to 0.
      */
      ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
    }

    cout << "x_ => " << ekf_.x_ << endl;

    // initializing previous timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // calc dt in seconds
  cout << "calc dt..." << endl;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.;
  cout << "dt = " << dt << endl;

  cout << "update 2D state transition matrix" << endl;
  // update 2D state transition matrix
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1., 0., dt, 0.,
		     0., 1., 0., dt,
			 0., 0., 1., 0.,
			 0., 0., 0., 1.;

  cout << "update process noise covariance matrix" << endl;
  // update process noise covariance matrix
  // noise = variance
  float process_variance_ax = 9;
  float process_variance_ay = 9;

  float dt_pow_2 = pow(dt, 2);
  float dt_pow_3 = pow(dt, 3);
  float dt_pow_4 = pow(dt, 4);

  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_(0,0) = dt_pow_4 * process_variance_ax / 4;
  ekf_.Q_(0,1) = 0.;
  ekf_.Q_(0,2) = dt_pow_3 * process_variance_ax / 2;
  ekf_.Q_(0,3) = 0.;
  ekf_.Q_(1,0) = 0.;
  ekf_.Q_(1,1) = dt_pow_4 * process_variance_ay / 4;
  ekf_.Q_(1,2) = 0.;
  ekf_.Q_(1,3) = dt_pow_3 * process_variance_ay / 2;
  ekf_.Q_(2,0) = dt_pow_3 * process_variance_ax / 2;
  ekf_.Q_(2,1) = 0.;
  ekf_.Q_(2,2) = dt_pow_2 * process_variance_ax;
  ekf_.Q_(2,3) = 0.;
  ekf_.Q_(3,0) = 0.;
  ekf_.Q_(3,1) = dt_pow_3 * process_variance_ay / 2;
  ekf_.Q_(3,2) = 0.;
  ekf_.Q_(3,3) = dt_pow_2 * process_variance_ay;

  cout << "ekf predict..." << endl;

  // predict
  ekf_.Predict();
  cout << "x_" << ekf_.x_ << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // Radar updates
	// set the measurement covariance matrix to radar (3x3)
	ekf_.R_ = R_radar_;

	// set the Hj matrix
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

	// update
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else
  {
    // Laser updates
	// set the measurement covariance matrix to laser (2x2)
	ekf_.R_ = R_laser_;

	// set the H matrix
	ekf_.H_ = H_laser_;

	// update
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
