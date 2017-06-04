#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/*
 * Constructor.
 */
FusionEKF::FusionEKF() 
{
    is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ <<		0.0225,		0,
					0,			0.0225;

	//measurement covariance matrix - radar
	R_radar_ <<		0.09,		0,			0,
					0,			0.0009,		0,
					0,			0,			0.09;

	// TODO: Finish initializing the FusionEKF. Set the process and measurement noises
	H_laser_ <<		1, 0, 0, 0,
					0, 1, 0, 0;

	// TODO: it makes only sense to initialize Hj_ after first state from ekf_ is available ?!?
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() 
{
}

void FusionEKF::Initialize(const MeasurementPackage &measurement_pack)
{
    /*****************************************************************************
    *  Initialization
    ****************************************************************************/

    // TODO: 1) Initialize the state ekf_.x_ with the first measurement.
    // TODO: 2) Create the covariance matrix.
    //			Remember: you'll need to convert radar from polar to cartesian coordinates.

    // first measurement
    //cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // TODO: Convert radar from polar to cartesian coordinates and initialize state.

        VectorXd cartesianPos = Polar2Cartesian(
            measurement_pack.raw_measurements_[0],
            measurement_pack.raw_measurements_[1],
            measurement_pack.raw_measurements_[2]);

        //printf("initial measurement R: (%.4f,%.4f)\n", cartesianPos[0], cartesianPos[1]);

        ekf_.x_[0] = cartesianPos[0]; // p_x
        ekf_.x_[1] = cartesianPos[1]; // p_y
        ekf_.x_[2] = 6.65259; // value from first radar measurement
        ekf_.x_[3] = 1.97674; // value from first radar measurement
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        // TODO: Initialize state.

        //printf("initial measurement L: (%.4f,%.4f)\n",
        //    measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1]);

        ekf_.x_[0] = measurement_pack.raw_measurements_[0];
        ekf_.x_[1] = measurement_pack.raw_measurements_[1];
        ekf_.x_[2] = 6.65259; // value from first radar measurement
        ekf_.x_[3] = 1.97674; // value from first radar measurement
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.F_ = MatrixXd(4, 4);   // values of F are calculated on each iteration
    ekf_.F_ <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    ekf_.Q_ = MatrixXd(4, 4);   // values of Q are calculated on each iteration
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  1,      0,      0,      0,
                0,      1,      0,      0,
                0,      0,      1000,   0,
                0,      0,      0,      1000;

    // done initializing, no need to predict or update
    is_initialized_ = true;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
	if (!is_initialized_) 
	{
        Initialize(measurement_pack);
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/
    //cout << "timestamp: " << measurement_pack.timestamp_ << endl;

    // TODO: Update the state transition matrix F according to the new elapsed time.
    //		  - Time is measured in seconds.

    if (0 == (measurement_pack.timestamp_ - previous_timestamp_))
    {
        printf("dt is zero!");
    }

    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    // TODO: Update the process noise covariance matrix.
    //       - Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    double noise_ax(9);
    double noise_ay(9);

    double dt_2 = pow(dt, 2);
    double dt_3 = pow(dt, 3);
    double dt_4 = pow(dt, 4);

    double dt_4_noise_ax = dt_4 * noise_ax / 4.0;
    double dt_3_noise_ax = dt_3 * noise_ax / 2.0;
    double dt_4_noise_ay = dt_4 * noise_ay / 4.0;
    double dt_3_noise_ay = dt_3 * noise_ay / 2.0;
    double dt_2_noise_ax = dt_2 * noise_ax;
    double dt_2_noise_ay = dt_2 * noise_ay;

    ekf_.Q_ <<  dt_4_noise_ax,  0,              dt_3_noise_ax,  0,
                0,              dt_4_noise_ay,  0,              dt_3_noise_ay,
                dt_3_noise_ax,  0,              dt_2_noise_ax,  0,
                0,              dt_3_noise_ay,  0,              dt_2_noise_ay;

	ekf_.Predict();

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

    // TODO: Use the sensor type to perform the update step.
    //       Update the state and covariance matrices.

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
	{
        //cout << "radar" << endl;

        // Radar updates
        VectorXd measurementVec(3);
        measurementVec << 
            measurement_pack.raw_measurements_[0],
            measurement_pack.raw_measurements_[1],
            measurement_pack.raw_measurements_[2];

        MatrixXd Hj = CalculateJacobian(ekf_.x_);
        ekf_.UpdateEKF(measurementVec, Hj, R_radar_);
	}
	else 
	{
        //cout << "laser" << endl;

        // Laser updates
        VectorXd measurementVec(2);
        measurementVec <<
            measurement_pack.raw_measurements_[0],
            measurement_pack.raw_measurements_[1];
        ekf_.Update(measurementVec, H_laser_, R_laser_);
	}

	// print the output
	//cout << "x_ = " << ekf_.x_ << endl;
	//cout << "P_ = " << ekf_.P_ << endl;
}
