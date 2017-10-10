#include <cmath>
#include <iostream>
#include <limits>
#include "constants.h"
#include "cost_functions.h"
#include "polynomial.h"

#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace std;

static const double SAFE_VEHICLE_DISTANCE = 1.5;
static const double EXPECTED_ACC_IN_ONE_SEC = 1;
static const double EXPECTED_JERK_IN_ONE_SEC = 2;
// 22.1 meters per second is 49.5 miles per hour.
static const double MAX_SPEED = 20;
// 13.4 meters per second is 30 miles per hour.
static const double MIN_SPEED = 13.4;
static const double MAX_ACCEL = 3;
static const double MAX_JERK = 3;
static const int TRAJECTORY_SAMPLES = 20;


CostFunctions::CostFunctions(const Vector3d& start_s,
                             const Vector3d& start_d,
                             const Trajectory& trajectory,
                             const Vehicle& target,
                             const VectorXd& delta,
                             double goal_t,
                             const std::vector<Vehicle>& vehicles)
  : start_s(start_s),
    start_d(start_d),
    trajectory(trajectory),
    target(target),
    delta(delta),
    goal_t(goal_t),
    vehicles(vehicles)
{
}

struct WeightedCostFunction
{
    std::function<double(CostFunctions*)> cost_function;
    double weight;
};

double CostFunctions::cost()
{
    using namespace std::placeholders;

    std::vector<WeightedCostFunction> cost_functions = {// {bind(&CostFunctions::time_diff_cost, _1), 1.0},
                                                        {bind(&CostFunctions::s_diff_cost, _1), 1.0},
                                                        {bind(&CostFunctions::d_diff_cost, _1), 1.0},
                                                        {bind(&CostFunctions::collision_cost, _1), 2.0},
                                                        {bind(&CostFunctions::buffer_cost, _1), 2.0},
                                                        {bind(&CostFunctions::efficiency_cost, _1), 1.0},
                                                        {bind(&CostFunctions::total_accel_cost, _1), 1.0},
                                                        {bind(&CostFunctions::total_jerk_cost, _1), 1.0},
                                                        {bind(&CostFunctions::max_speed_cost, _1), 2.0},
                                                        {bind(&CostFunctions::max_s_accel_cost, _1), 2.0},
                                                        {bind(&CostFunctions::max_s_jerk_cost, _1), 1.5},
                                                        {bind(&CostFunctions::max_d_accel_cost, _1), 2.0},
                                                        {bind(&CostFunctions::max_d_jerk_cost, _1), 1.5},
                                                        // {bind(&CostFunctions::offroad_cost, _1), 1.0},
                                                        // {bind(&CostFunctions::offcenter_cost, _1), 1.0},
                                                        {bind(&CostFunctions::backward_cost, _1), 1.0}
                                                       };

    double cost = 0;
    for (const auto& wcf : cost_functions)
    {
        cost += wcf.weight * wcf.cost_function(this);
    }
    return cost;
}

double logistic(double x)
{
    return 2.0 / (1.0 + exp(-x)) - 1.0;
}

double CostFunctions::time_diff_cost()
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

double CostFunctions::s_diff_cost()
{
    VectorXd target_state = target.state_in(goal_t) + delta;
    Vector3d s_target = target_state.head(3);

    return state_diff_cost(goal_t, s_target, trajectory.s_coefficients, SIGMA_S);
}

double CostFunctions::d_diff_cost()
{
    VectorXd target_state = target.state_in(goal_t) + delta;
    Vector3d d_target = target_state.tail(3);

    return state_diff_cost(goal_t, d_target, trajectory.d_coefficients, SIGMA_D);
}

double CostFunctions::collision_cost()
{
    double nearest = trajectory.nearest_approach(vehicles);

    return (nearest < SAFE_VEHICLE_DISTANCE) ? 1.0 : 0.0;
}

double CostFunctions::buffer_cost()
{
    double nearest = trajectory.nearest_approach(vehicles);

    return logistic(3 * SAFE_VEHICLE_DISTANCE / nearest);
}

double CostFunctions::efficiency_cost()
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
    const double dt = goal_t / TRAJECTORY_SAMPLES;
    for (int i = 0; i < TRAJECTORY_SAMPLES; i++)
    {
        double t = dt * i;
        double value = derivative.evaluate(t);
        total_value += abs(value * dt);
    }
    double value_per_second = total_value / goal_t;
    return logistic(value_per_second / expected_value_per_second);
}

double CostFunctions::total_accel_cost()
{
    return total_derivative_cost(trajectory.s_coefficients, 2, goal_t, EXPECTED_ACC_IN_ONE_SEC);
}

double CostFunctions::total_jerk_cost()
{
    return total_derivative_cost(trajectory.s_coefficients, 3, goal_t, EXPECTED_JERK_IN_ONE_SEC);
}

double max_derivative_cost(const VectorXd& trajectory_coefficients,
                           int derivative_count,
                           double goal_t,
                           double max,
                           double min = numeric_limits<double>::min())
{
    auto function_and_derivatives = get_function_and_derivatives(trajectory_coefficients, derivative_count);
    auto derivative = function_and_derivatives[derivative_count];

    const double dt = goal_t / TRAJECTORY_SAMPLES;
    for (int i = 0; i < TRAJECTORY_SAMPLES; i++)
    {
        double t = dt * i;
        double value = derivative.evaluate(t);
        if (abs(value) > max)
        {
            return 1;
        }
        if (value < min)
        {
            return 1;
        }
    }
    return 0;
}

double CostFunctions::max_speed_cost()
{
    return max_derivative_cost(trajectory.s_coefficients, 1, goal_t, MAX_SPEED, MIN_SPEED);
}

double CostFunctions::max_s_accel_cost()
{
    return max_derivative_cost(trajectory.s_coefficients, 2, goal_t, MAX_ACCEL);
}

double CostFunctions::max_s_jerk_cost()
{
    return max_derivative_cost(trajectory.s_coefficients, 3, goal_t, MAX_JERK);
}

double CostFunctions::max_d_accel_cost()
{
    return max_derivative_cost(trajectory.d_coefficients, 2, goal_t, MAX_ACCEL);
}

double CostFunctions::max_d_jerk_cost()
{
    return max_derivative_cost(trajectory.d_coefficients, 3, goal_t, MAX_JERK);
}

double CostFunctions::offroad_cost()
{
    static const double MIN_D = 0;
    static const double MAX_D = 12;
    return max_derivative_cost(trajectory.d_coefficients, 0, goal_t, MAX_D, MIN_D);
}

double CostFunctions::offcenter_cost()
{
    Polynomial d_trajectory(trajectory.d_coefficients);

    static const double dt = goal_t / TRAJECTORY_SAMPLES;
    static const int lane_width = 4;
    static const double lane_center = lane_width / 2;

    double total_offcenter = 0;
    for (int i = 1; i < TRAJECTORY_SAMPLES; i++)
    {
        double t = dt * i;
        double d = d_trajectory.evaluate(t);
        double d_in_lane = (d - (static_cast<int>(d) / lane_width) * lane_width);
        // cout << "d:" << d << ",d_in_lane:" << d_in_lane << endl;
        double offcenter = abs(lane_center - d_in_lane) / lane_center;
        total_offcenter += offcenter;
    }
    return total_offcenter / TRAJECTORY_SAMPLES;
}

double CostFunctions::backward_cost()
{
    Polynomial s_trajectory(trajectory.s_coefficients);

    const double dt = goal_t / TRAJECTORY_SAMPLES;

    double prev_value = s_trajectory.evaluate(0);
    for (int i = 1; i < TRAJECTORY_SAMPLES; i++)
    {
        double t = dt * i;
        double value = s_trajectory.evaluate(t);
        if (value < prev_value)
        {
            return 1;
        }
        prev_value = value;
    }
    return 0;
}
