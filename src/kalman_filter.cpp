#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
   //measurement covariance matrix for laser
   R_laser = MatrixXd(2, 2);
   R_laser << 0.0225, 0,
	   0, 0.0225;

   //measurement covariance matrix for radar
   R_radar = MatrixXd(3, 3);
   R_radar << 0.09, 0, 0,
	   0, 0.0009, 0,
	   0, 0, 0.09;

   //measurement matrix for laser
   H_laser = MatrixXd(2, 4);
   H_laser << 1, 0, 0, 0,
	   0, 1, 0, 0;

   //measurement matrix for radar
   H_j = MatrixXd(3, 4);
   H_j << 0, 0, 0, 0,
	   1e-9, 1e-9, 0, 0,
	   0, 0, 0, 0;	
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  //H_ = H_in;
  //R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  	VectorXd z_pred = H_laser * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_laser.transpose();
	MatrixXd S = H_laser * P_ * Ht + R_laser;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Tools t;
  MatrixXd H_j = t.CalculateJacobian(x_);
  
  MatrixXd y = z - t.h(x_);
  MatrixXd S = H_j * P_ * H_j.transpose() + R_radar;
  MatrixXd K = P_ * H_j.transpose() * S.inverse();
  
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_j) * P_;
  
}
