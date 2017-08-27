#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <functional>
#include "Eigen-3.3/Eigen/Dense"
#include "vehicle.h"

using Eigen::VectorXd;

struct Trajectory;

typedef std::function<double(const Trajectory& trajectory,
                             int target_vehicle,
                             const VectorXd& delta,
                             double goal_t,
                             const std::vector<Vehicle>& vehicles)> CostFunction;

struct WeightedCostFunction
{
    CostFunction cost_function;
    double weight;
};

struct Trajectory
{
    VectorXd s_coefficients;
    VectorXd d_coefficients;
    double t;

    double calculate_cost(int target_vehicle,
                          const VectorXd& delta,
                          double goal_t,
                          const std::vector<Vehicle>& vehicles,
                          const std::vector<WeightedCostFunction>& cost_functions);
};

#endif
