#include <cmath>
#include <limits>
#include "polynomial.h"
#include "trajectory.h"
#include "util.h"

using namespace std;

double Trajectory::calculate_cost(const Vehicle& target,
                                  const VectorXd& delta,
                                  double goal_t,
                                  const std::vector<Vehicle>& vehicles,
                                  const std::vector<WeightedCostFunction>& cost_functions) const
{
    double cost = 0;
    for (const WeightedCostFunction& wcf : cost_functions)
    {
        cost += wcf.weight * wcf.cost_function(*this,
                                               target,
                                               delta,
                                               goal_t,
                                               vehicles);
    }
    return cost;
}

double Trajectory::nearest_approach(const std::vector<Vehicle>& vehicles) const
{
    double nearest = numeric_limits<double>::max();
    for (const auto& v : vehicles)
    {
        double dist = nearest_approach(v);
        if (dist < nearest)
        {
            nearest = dist;
        }
    }
    return nearest;
}

double Trajectory::nearest_approach(const Vehicle& vehicle) const
{
    double nearest = numeric_limits<double>::max();

    Polynomial s(s_coefficients);
    Polynomial d(d_coefficients);

    const int increment_count = 100;

    for (int i = 0; i < increment_count; i++)
    {
        double t = static_cast<double>(i) / 100 * this->t;
        double s_trajectory = s.evaluate(t);
        double d_trajectory = d.evaluate(t);

        VectorXd state = vehicle.state_in(t);
        double s_vehicle = state[0];
        double d_vehicle = state[3];

        double dist = distance(s_trajectory, s_vehicle, d_trajectory, d_vehicle);
        if (dist < nearest)
        {
            nearest = dist;
        }
    }

    return nearest;
}


Polynomial Trajectory::s_poly() const
{
    return Polynomial(s_coefficients);
}

Polynomial Trajectory::d_poly() const
{
    return Polynomial(d_coefficients);
}

bool TrajectoryCost::operator<(const TrajectoryCost& other) const
{
    return cost < other.cost;
}
