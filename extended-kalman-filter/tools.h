#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobianAvoge(const VectorXd& x_state);

  /**
  * A helper method to convert the state matrix into the radar form
  */
  VectorXd State2radar(const VectorXd& x_state);

  /**
  * A helper method to convert the radar matrix into the state form
  */
  VectorXd Radar2state(const VectorXd& x_radar);


};

#endif /* TOOLS_H_ */
