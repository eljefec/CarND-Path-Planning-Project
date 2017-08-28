#ifndef PTG_H
#define PTG_H

#include "trajectory.h"
#include "vehicle.h"
#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;

double time_diff_cost(const Trajectory& trajectory,
                      int target_vehicle,
                      const VectorXd& delta,
                      double goal_t,
                      const std::vector<Vehicle>& vehicles);

double s_diff_cost(const Trajectory& trajectory,
                   int target_vehicle,
                   const VectorXd& delta,
                   double goal_t,
                   const std::vector<Vehicle>& vehicles);

#endif