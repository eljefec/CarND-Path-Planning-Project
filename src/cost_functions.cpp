#include "cost_functions.h"
#include <cmath>

using namespace std;

double logistic(double x)
{
    return 2.0 / (1.0 + exp(-x)) - 1.0;
}

double time_diff_cost(const Trajectory& trajectory,
                      int target_vehicle,
                      const VectorXd& delta,
                      double goal_t,
                      const std::vector<Vehicle>& vehicles)
{
    return logistic(abs(trajectory.t - goal_t) / goal_t);
}
