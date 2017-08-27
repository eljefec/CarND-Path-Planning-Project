#ifndef PTG_H
#define PTG_H

#include "trajectory.h"
#include "vehicle.h"

#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;

Trajectory PTG(const VectorXd& start_s,
               const VectorXd& start_d,
               int target_vehicle,
               const VectorXd& delta,
               double T,
               const std::vector<Vehicle>& vehicles);

#endif
