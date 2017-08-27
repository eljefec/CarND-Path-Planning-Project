#include "trajectory.h"

double Trajectory::calculate_cost(int target_vehicle,
                                  const VectorXd& delta,
                                  double goal_t,
                                  const std::vector<Vehicle>& vehicles,
                                  const std::vector<WeightedCostFunction>& cost_functions) const
{
    double cost = 0;
    for (const WeightedCostFunction& wcf : cost_functions)
    {
        cost += wcf.weight * wcf.cost_function(*this,
                                               target_vehicle,
                                               delta,
                                               goal_t,
                                               vehicles);
    }
    return cost;
}

bool TrajectoryCost::operator<(const TrajectoryCost& other) const
{
    return cost < other.cost;
}
