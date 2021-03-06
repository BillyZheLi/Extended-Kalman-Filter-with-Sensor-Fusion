#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PI 3.14159265358979323846

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred = VectorXd(3);
  z_pred << sqrt(x_(0)*x_(0) + x_(1)*x_(1)),
            atan2(x_(1), x_(0)),
            (x_(0)*x_(2) + x_(1)*x_(3)) / sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  
  VectorXd y = z - z_pred;
  //wrap delta theta onto [-pi, pi]
  while (y(1) < -PI || y(1) > PI) {
    if (y(1) < -PI) {
      y(1) += 2*PI;
    } else {
      y(1) -= 2*PI;
    }
  }
  
  // only correct when jacobian can be calculated
  if ( !std::isnan(H_(0,0)) ) {  
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
  } 
}
