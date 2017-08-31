#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <functional>
#include "Eigen-3.3/Eigen/Dense"
#include "polynomial.h"
#include "vehicle.h"

using Eigen::VectorXd;

struct Trajectory;

typedef std::function<double(const Trajectory& trajectory,
                             const Vehicle& target,
                             const VectorXd& delta,
                             double goal_t,
                             const std::vector<Vehicle>& vehicles)> CostFunction;

struct Trajectory
{
    VectorXd s_coefficients;
    VectorXd d_coefficients;
    double t;

    double nearest_approach(const std::vector<Vehicle>& vehicles) const;

    Polynomial s_poly() const;
    Polynomial d_poly() const;

private:
    double nearest_approach(const Vehicle& vehicle) const;
};

struct TrajectoryCost
{
    Trajectory trajectory;
    double cost;

    bool operator<(const TrajectoryCost& other) const;
};

#endif
