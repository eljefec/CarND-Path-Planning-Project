#ifndef PTG_H
#define PTG_H

#include <vector>
#include "trajectory.h"
#include "vehicle.h"

#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;

struct PTG_Goal
{
    Vehicle target;
    VectorXd delta;
    double T;
};

Trajectory PTG(const VectorXd& start_s,
               const VectorXd& start_d,
               const std::vector<PTG_Goal>& goals,
               const std::vector<Vehicle>& vehicles);

#endif
