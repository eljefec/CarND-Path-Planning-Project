#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include "trajectory.h"
#include "vehicle.h"

#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;

class Polynomial
{
public:
    Polynomial(const VectorXd& coefficients);

    Polynomial differentiate();

    double evaluate(double t);

private:
    VectorXd coefficients;
};

#endif
