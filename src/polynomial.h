#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;

class Polynomial
{
public:
    Polynomial(const VectorXd& coefficients);

    Polynomial differentiate() const;

    double evaluate(double t) const;

private:
    VectorXd coefficients;
};

#endif
