#include <iostream>

#include "kalman_filter.h"
#include "tools.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


KalmanFilter::KalmanFilter() 
{
}

KalmanFilter::~KalmanFilter() 
{
}

void KalmanFilter::Init(
	VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
	MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 
{
    // TODO: unused?!?
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	//H_ = H_in;
	//R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() 
{
	// TODO: predict the state
    x_ = F_ * x_;

    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

    //cout << "x: " << x_ << endl;
    //cout << "F: " << F_ << endl;
}

void KalmanFilter::Update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd R)
{
	// TODO: update the state by using Kalman Filter equations

    VectorXd z_pred = H * x_;
    VectorXd y = z - z_pred;

    MatrixXd Ht = H.transpose();
    MatrixXd S = H * P_ * Ht + R;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const Eigen::MatrixXd& H, const Eigen::MatrixXd R)
{
	// TODO: update the state by using Extended Kalman Filter equations

    VectorXd z_pred = h_of_x(x_); //VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;

    MatrixXd Ht = H.transpose();
    MatrixXd S = H * P_ * Ht + R;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H) * P_;
}
