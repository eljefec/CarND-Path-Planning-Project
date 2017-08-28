#include <cmath>
#include "constants.h"
#include "cost_functions.h"
#include "polynomial.h"

#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace std;

double logistic(double x)
{
    return 2.0 / (1.0 + exp(-x)) - 1.0;
}

double time_diff_cost(const Trajectory& trajectory,
                      int target_vehicle,
                      const VectorXd& delta,
                      double goal_t,
                      const std::vector<Vehicle>& vehicles)
{
    return logistic(abs(trajectory.t - goal_t) / goal_t);
}

std::vector<Polynomial> get_function_and_derivatives(const VectorXd& coeffs, int derivative_count)
{
    std::vector<Polynomial> polynomials;
    polynomials.emplace_back(Polynomial{coeffs});

    for (int i = 0; i < derivative_count; i++)
    {
        polynomials.push_back(polynomials[i].differentiate());
    }

    return polynomials;
}

double s_diff_cost(const Trajectory& trajectory,
                   int target_vehicle,
                   const VectorXd& delta,
                   double goal_t,
                   const std::vector<Vehicle>& vehicles)
{
    VectorXd target_state = vehicles[target_vehicle].state_in(goal_t) + delta;
    Vector3d s_target = target_state.head(3);

    auto function_and_derivatives = get_function_and_derivatives(trajectory.s_coefficients, 2);
    Vector3d s_trajectory;
    for (int i = 0; i < 3; i++)
    {
        s_trajectory[i] = function_and_derivatives[i].evaluate(goal_t);
    }

    double cost = 0;
    for (int i = 0; i < 3; i++)
    {
        double diff = abs(s_trajectory[i] - s_target[i]);
        cost += logistic(diff / SIGMA_S[i]);
    }
    return cost;
}
