#include "trajectory.h"

double Trajectory::calculate_cost(int target_vehicle,
                                  const VectorXd& delta,
                                  double goal_t,
                                  const std::vector<Vehicle>& vehicles,
                                  const std::vector<WeightedCostFunction>& cost_functions)
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
