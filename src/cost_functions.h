#ifndef PTG_H
#define PTG_H

#include "trajectory.h"
#include "vehicle.h"
#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;

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

    double cost();

private:
    double time_diff_cost();
    double s_diff_cost();
    double d_diff_cost();
    double collision_cost();
    double buffer_cost();
    double efficiency_cost();
    double total_accel_cost();
    double total_jerk_cost();
    double max_speed_cost();
    double max_accel_cost();
    double max_jerk_cost();
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
};

#endif
