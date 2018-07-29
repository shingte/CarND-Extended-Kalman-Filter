#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  for (std::size_t i = 0; i != estimations.size(); ++i) {
	VectorXd residual = estimations[i] - ground_truth[i];

	//coefficient-wise multiplication
	residual = residual.array()*residual.array();

	rmse += residual;
  }

  //calculate mean value
  rmse /= estimations.size();

  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  double px = x_state[0];
  double py = x_state[1];
  double vx = x_state[2];
  double vy = x_state[3];

  double c1 = px*px + py*py;
  //avoid division by zero
  if (c1 < 1e-12 ) { c1 = 1e-12; }
  double c2 = std::sqrt(c1);
  double c3 = c1*c2;

  MatrixXd h_j(3, 4);

  //compute the Jacobian matrix
  h_j << px/c2, py/c2, 0, 0,
      -py/c1, px/c1, 0, 0,
      py*(py*vx - px*vy)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return h_j;
}
