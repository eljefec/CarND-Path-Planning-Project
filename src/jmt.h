#ifndef POLY_SOLVER_H
#define POLY_SOLVER_H

#include "Eigen-3.3/Eigen/Dense"

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

class JMT
{
public:
    JMT(double T);

    VectorXd solve(const Vector3d& start, const Vector3d& end) const;

private:
    double T;
    double T2;
    Matrix3d A_inverse;
};

#endif
