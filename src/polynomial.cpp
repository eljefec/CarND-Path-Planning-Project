#include "polynomial.h"

Polynomial::Polynomial(const VectorXd& coefficients)
  : coefficients(coefficients)
{
}

Polynomial Polynomial::differentiate()
{
    VectorXd derivative(coefficients.size() - 1);
    for (int i = 1; i < coefficients.size(); i++)
    {
        derivative[i - 1] = i * coefficients[i];
    }
    return Polynomial(derivative);
}

double Polynomial::evaluate(double t)
{
    double total = 0;
    for (int i = 0; i < coefficients.size(); i++)
    {
        total += coefficients[i] * pow(t, i);
    }
    return total;
}
