#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"


/**
* A helper method to calculate RMSE.
*/
Eigen::VectorXd CalculateRMSE(
    const std::vector<Eigen::VectorXd> &estimations,
    const std::vector<Eigen::VectorXd> &ground_truth);

/**
* A helper method to calculate Jacobians.
*/
Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

/**
 * calculate measurement function h(x) for equation y = z - h(x)
 * @param x input 4D state vector (px, py, vx, vy)
 * @param return 3D vector of state in measurement space
 */
Eigen::VectorXd h_of_x(const Eigen::VectorXd& x);

/**
 * map given 3D polar measurement to 4D cartesian coordinates (px, py, vx, vy)
 * @param rho range
 * @param phi bearing
 * @param rho_dot radial velocity, range rate
 * @return 4D vector (px, py, vx, vy)
 */
Eigen::VectorXd Polar2Cartesian(
    const double rho, const double phi, const double rho_dot);

#endif /* TOOLS_H_ */
