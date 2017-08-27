#ifndef POLY_SOLVER_H
#define POLY_SOLVER_H

#include "Eigen-3.3/Eigen/Dense"

using Eigen::Vector3d;
using Eigen::VectorXd;

VectorXd JMT(const Vector3d& start, const Vector3d& end, double T);

#endif
