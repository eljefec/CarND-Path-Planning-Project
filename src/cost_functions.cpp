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

double state_diff_cost(double goal_t,
                       const Vector3d& target_state,
                       const Vector3d& trajectory_coefficients,
                       const Vector3d& stddev)
{
    auto function_and_derivatives = get_function_and_derivatives(trajectory_coefficients, 2);

    Vector3d trajectory;
    for (int i = 0; i < 3; i++)
    {
        trajectory[i] = function_and_derivatives[i].evaluate(goal_t);
    }

    double cost = 0;
    for (int i = 0; i < 3; i++)
    {
        double diff = abs(trajectory[i] - target_state[i]);
        cost += logistic(diff / stddev[i]);
    }
    return cost;
}

double s_diff_cost(const Trajectory& trajectory,
                   int target_vehicle,
                   const VectorXd& delta,
                   double goal_t,
                   const std::vector<Vehicle>& vehicles)
{
    VectorXd target_state = vehicles[target_vehicle].state_in(goal_t) + delta;
    Vector3d s_target = target_state.head(3);

    return state_diff_cost(goal_t, s_target, trajectory.s_coefficients, SIGMA_S);
}

double d_diff_cost(const Trajectory& trajectory,
                   int target_vehicle,
                   const VectorXd& delta,
                   double goal_t,
                   const std::vector<Vehicle>& vehicles)
{
    VectorXd target_state = vehicles[target_vehicle].state_in(goal_t) + delta;
    Vector3d d_target = target_state.tail(3);

    return state_diff_cost(goal_t, d_target, trajectory.d_coefficients, SIGMA_D);
}
