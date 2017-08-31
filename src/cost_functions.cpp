#include <cmath>
#include "constants.h"
#include "cost_functions.h"
#include "polynomial.h"

#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace std;

static const double SAFE_VEHICLE_DISTANCE = 3;
static const double EXPECTED_ACC_IN_ONE_SEC = 1;
static const double EXPECTED_JERK_IN_ONE_SEC = 2;
static const double MAX_ACCEL = 10;
static const double MAX_JERK = 10;

double logistic(double x)
{
    return 2.0 / (1.0 + exp(-x)) - 1.0;
}

double time_diff_cost(const Trajectory& trajectory,
                      const Vehicle& target,
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
                       const VectorXd& trajectory_coefficients,
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
                   const Vehicle& target,
                   const VectorXd& delta,
                   double goal_t,
                   const std::vector<Vehicle>& vehicles)
{
    VectorXd target_state = target.state_in(goal_t) + delta;
    Vector3d s_target = target_state.head(3);

    return state_diff_cost(goal_t, s_target, trajectory.s_coefficients, SIGMA_S);
}

double d_diff_cost(const Trajectory& trajectory,
                   const Vehicle& target,
                   const VectorXd& delta,
                   double goal_t,
                   const std::vector<Vehicle>& vehicles)
{
    VectorXd target_state = target.state_in(goal_t) + delta;
    Vector3d d_target = target_state.tail(3);

    return state_diff_cost(goal_t, d_target, trajectory.d_coefficients, SIGMA_D);
}

double collision_cost(const Trajectory& trajectory,
                      const Vehicle& target,
                      const VectorXd& delta,
                      double goal_t,
                      const std::vector<Vehicle>& vehicles)
{
    double nearest = trajectory.nearest_approach(vehicles);

    return (nearest < SAFE_VEHICLE_DISTANCE) ? 1.0 : 0.0;
}

double buffer_cost(const Trajectory& trajectory,
                   const Vehicle& target,
                   const VectorXd& delta,
                   double goal_t,
                   const std::vector<Vehicle>& vehicles)
{
    double nearest = trajectory.nearest_approach(vehicles);

    return logistic(SAFE_VEHICLE_DISTANCE / nearest);
}

double efficiency_cost(const Trajectory& trajectory,
                       const Vehicle& target,
                       const VectorXd& delta,
                       double goal_t,
                       const std::vector<Vehicle>& vehicles)
{
    Polynomial s_trajectory(trajectory.s_coefficients);
    double v_trajectory = s_trajectory.evaluate(trajectory.t) / trajectory.t;

    VectorXd target_state = target.state_in(goal_t);
    double s_target = target_state[0];
    double v_target = s_target / trajectory.t;

    return logistic(2.0 * (v_target - v_trajectory) / v_trajectory);
}

double total_derivative_cost(const VectorXd& trajectory_coefficients,
                             int derivative_count,
                             double goal_t,
                             double expected_value_per_second)
{
    auto function_and_derivatives = get_function_and_derivatives(trajectory_coefficients, derivative_count);
    auto derivative = function_and_derivatives[derivative_count];

    double total_value = 0;
    double dt = goal_t / 100.0;
    for (int i = 0; i < 100; i++)
    {
        double t = dt * i;
        double value = derivative.evaluate(t);
        total_value += abs(value * dt);
    }
    double value_per_second = total_value / goal_t;
    return logistic(value_per_second / expected_value_per_second);
}

double total_accel_cost(const Trajectory& trajectory,
                        const Vehicle& target,
                        const VectorXd& delta,
                        double goal_t,
                        const std::vector<Vehicle>& vehicles)
{
    return total_derivative_cost(trajectory.s_coefficients, 2, goal_t, EXPECTED_ACC_IN_ONE_SEC);
}

double total_jerk_cost(const Trajectory& trajectory,
                       const Vehicle& target,
                       const VectorXd& delta,
                       double goal_t,
                       const std::vector<Vehicle>& vehicles)
{
    return total_derivative_cost(trajectory.s_coefficients, 3, goal_t, EXPECTED_JERK_IN_ONE_SEC);
}

double max_derivative_cost(const VectorXd& trajectory_coefficients,
                           int derivative_count,
                           double goal_t,
                           double max)
{
    auto function_and_derivatives = get_function_and_derivatives(trajectory_coefficients, derivative_count);
    auto derivative = function_and_derivatives[derivative_count];

    double max_value = 0;
    double dt = goal_t / 100.0;
    for (int i = 0; i < 100; i++)
    {
        double t = dt * i;
        double value = derivative.evaluate(t);
        if (abs(value) > max)
        {
            return 1;
        }
    }
    return 0;
}

double max_accel_cost(const Trajectory& trajectory,
                      const Vehicle& target,
                      const VectorXd& delta,
                      double goal_t,
                      const std::vector<Vehicle>& vehicles)
{
    return max_derivative_cost(trajectory.s_coefficients, 2, goal_t, MAX_ACCEL);
}

double max_jerk_cost(const Trajectory& trajectory,
                     const Vehicle& target,
                     const VectorXd& delta,
                     double goal_t,
                     const std::vector<Vehicle>& vehicles)
{
    return max_derivative_cost(trajectory.s_coefficients, 3, goal_t, MAX_JERK);
}
