#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;

struct Trajectory
{
    VectorXd s_coefficients;
    VectorXd d_coefficients;
    double t;
};

#endif
