#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.size() == 0 or estimations.size() != ground_truth.size()) {
    cout << "Error: Check the size of estimations vector and ground_trth vector." << endl;
    return rmse;
  }

  // calculate the squared sum
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd diff = estimations[i] - ground_truth[i];
    rmse = rmse.array() + diff.array() * diff.array();
  } 

  // calculate the mean and then sqrt
  rmse = rmse.array() / estimations.size();
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float p11 = px*px + py*py;
  float p12 = sqrt(p11);
  float p32 = pow(p12, 3);
  
  float almostZero = 0.000001;
  // calcualte each element of the jacobian matrix
  if (p11 > almostZero || p11 < -almostZero) {
    Hj << px/p12, py/p12, 0, 0,
          -py/p11, px/p11, 0, 0,
          py*(vx*py-vy*px)/p32, px*(vy*px-vx*py)/p32, px/p12, py/p12;
  } else {
    cout << "Warning: px and py are too smal to calculate jacobian " << endl;
    Hj << std::nan(""), std::nan(""), 0, 0,
          std::nan(""), std::nan(""), 0, 0,
          std::nan(""), std::nan(""), std::nan(""), std::nan("");
  }

  return Hj;
}
