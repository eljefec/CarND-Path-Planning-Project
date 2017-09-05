#ifndef PTG_H
#define PTG_H

#include <limits>
#include "trajectory.h"
#include "vehicle.h"
#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;

struct Evaluation
{
    bool feasible;
    double cost;
};

class CostFunctions
{
public:
    CostFunctions(const Vector3d& start_s,
                  const Vector3d& start_d,
                  const Trajectory& trajectory,
                  const Vehicle& target,
                  const VectorXd& delta,
                  double goal_t,
                  const std::vector<Vehicle>& vehicles);

    Evaluation evaluate();

private:
    double max_derivative_cost(const VectorXd& trajectory_coefficients,
                               int derivative_count,
                               double goal_t,
                               double max,
                               double min = std::numeric_limits<double>::min());

    double time_diff_cost();
    double s_diff_cost();
    double d_diff_cost();
    double collision_cost();
    double buffer_cost();
    double efficiency_cost();
    double total_accel_cost();
    double total_jerk_cost();
    double max_speed_cost();
    double max_s_accel_cost();
    double max_s_jerk_cost();
    double max_d_accel_cost();
    double max_d_jerk_cost();
    double offroad_cost();
    double offcenter_cost();
    double backward_cost();

    const Vector3d& start_s;
    const Vector3d& start_d;
    const Trajectory& trajectory;
    const Vehicle& target;
    const VectorXd& delta;
    double goal_t;
    const std::vector<Vehicle>& vehicles;

    bool feasible;
};

#endif
