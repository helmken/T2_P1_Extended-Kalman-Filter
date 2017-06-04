#include <iostream>
#include "tools.h"


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;


VectorXd CalculateRMSE(
	const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) 
{
	// TODO: Calculate the RMSE here.

    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    // ... your code here
    if (    estimations.size() != ground_truth.size()
        ||  estimations.empty())
    {
        cout << "CalculateRMSE: invalid sizes of estimations and ground truths" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (size_t i = 0; i < estimations.size(); ++i)
    {
        // ... your code here
        const VectorXd &curEstimate = estimations[i];
        const VectorXd &curGroundTruth = ground_truth[i];

        if (    curEstimate.size() != curGroundTruth.size()
            ||  0 == curEstimate.size())
        {
            cout << "CalculateRMSE: invalid size of estimation or ground truth" << endl;
            return rmse;
        }

        VectorXd residual = curEstimate - curGroundTruth;
        VectorXd residual_sq = residual.array() * residual.array();

        rmse += residual_sq;
    }

    //calculate the mean
    // ... your code here
    rmse = rmse / estimations.size();

    //calculate the squared root
    // ... your code here
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) 
{
	// TODO: Calculate a Jacobian here.

    MatrixXd Hj(3, 4);

    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    double px_py_sq_sum = px*px + py*py;
    double sqrt_px_py_sq_sum = sqrt(px_py_sq_sum);
    double sqrt3_px_py_sq_sum = px_py_sq_sum * sqrt_px_py_sq_sum;

    //check division by zero
    if (fabs(px_py_sq_sum) < 0.0001)
    {
        cout << "***\n***CalculateJacobian: computation not possible: division by zero***\n***\n";
        return Hj;
    }

    //compute the Jacobian matrix
    Hj(0, 0) = px / sqrt_px_py_sq_sum;
    Hj(0, 1) = py / sqrt_px_py_sq_sum;
    Hj(0, 2) = 0.0;
    Hj(0, 3) = 0.0;

    Hj(1, 0) = -py / px_py_sq_sum;
    Hj(1, 1) = px / px_py_sq_sum;
    Hj(1, 2) = 0.0;
    Hj(1, 3) = 0.0;

    Hj(2, 0) =   py * (vx * py - vy * px)
               / sqrt3_px_py_sq_sum;
    Hj(2, 1) =   px * (vy * px - vx * py)
               / sqrt3_px_py_sq_sum;
    Hj(2, 2) = px / sqrt_px_py_sq_sum;
    Hj(2, 3) = py / sqrt_px_py_sq_sum;

    return Hj;
}

VectorXd h_of_x(const VectorXd& x)
{
    VectorXd result = VectorXd(3);

    if (x.size() != 4)
    {
        cout << "ERROR: h_of_x: unexpected input vector size\n";
        return result;
    }

    double px = x[0];
    double py = x[1];
    double vx = x[2];
    double vy = x[3];

    //if (fabs(py) < 0.1)
    //{
    //    // values of py < 0.1 flip sign of angle
    //    py = 0.1;
    //}

    if (fabs(px) < 0.001)
    {
        cout << "ERROR: h_of_x: fabs(px) < 0.0001\n";
        return result;
    }

    double sqrt_px2_plus_py2 = sqrt(px * px + py * py);

    result[0] = sqrt_px2_plus_py2;
    
    double angle = atan2(py, px);
    if (angle <= -M_PI || angle >= M_PI)
    {
        cout << "angle is not in range -pi .. pi\n";
    }
    //else
    //{
    //    printf("angle=%.3f | %.3f, x=%.3f, y=%.3f, y/x=%.3f\n", angle * 180 / M_PI, angle, px, py, py/px);
    //}

    result[1] = angle;

    result[2] = (px * vx + py * vy) / sqrt_px2_plus_py2;

    return result;
}

Eigen::VectorXd Polar2Cartesian(
    const double rho, const double phi, const double rho_dot)
{
    VectorXd result = VectorXd(4);
    
    result[0] = rho * cos(phi); // p_x
    result[1] = rho * sin(phi); // p_y
    result[2] = 0.0; // can't calculate velocity
    result[3] = 0.0; // can't calculate velocity
    
    return result;
}
