#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Hj_in, MatrixXd &R_laser_in, 
						MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
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
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	
	// new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K*H_) * P_;
	
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
    * update the state by using Extended Kalman Filter equations
	*/
	float p_x = x_[0];
	float p_y = x_[1];
	float v_x = x_[2];
	float v_y = x_[3];
	
	VectorXd z_pred = VectorXd(3);
	z_pred << sqrt(p_x*p_x + p_y*p_y),
	          atan2(p_y, p_x),
			  (p_x*v_x + p_y*v_y)/(sqrt(p_x*p_x + p_y*p_y));
	
	cout << z[1] << " | " << z_pred[1] << " | " << p_y << endl;
	VectorXd y = z - z_pred;
	y[1] = fmod(y[1], (2*M_PI));
	//cout << "After y" << endl;
	MatrixXd Hjt = Hj_.transpose();
	MatrixXd S = Hj_ * P_ * Hjt + R_radar_;
	MatrixXd Si = S.inverse();
	MatrixXd PHjt = P_ * Hjt;
	MatrixXd K = PHjt * Si;
	
	// new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K*Hj_) * P_;

}
