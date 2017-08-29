#include <algorithm>
#include <random>
#include "constants.h"
#include "cost_functions.h"
#include "poly_solver.h"
#include "ptg.h"

using namespace std;

vector<VectorXd> perturb_state(const VectorXd& state, int sample_count)
{
    static default_random_engine generator(17);

    VectorXd stddevs(6);
    stddevs << SIGMA_S, SIGMA_D;

    vector<normal_distribution<double>> distributions;
    for (size_t i = 0; i < stddevs.size(); i++)
    {
        distributions.emplace_back(normal_distribution<double>(state[i], stddevs[i]));
    }

    vector<VectorXd> states;
    for (int i = 0; i < sample_count; i++)
    {
        VectorXd new_state(6);
        for (int i = 0; i < 6; i++)
        {
            new_state[i] = distributions[i](generator);
        }
        states.push_back(new_state);
    }

    return states;
}

struct Goal
{
    Eigen::Vector3d s;
    Eigen::Vector3d d;
    double t;
};

vector<Goal> perturb_goal(const VectorXd& target_state, int sample_count, double t)
{
    auto states = perturb_state(target_state, sample_count);
    vector<Goal> goals;
    for (const VectorXd& state : states)
    {
        goals.emplace_back(Goal{state.head(3), state.tail(3), t});
    }
    return goals;
}

vector<Goal> generate_goals(const Vehicle& target,
                            const VectorXd& delta,
                            double T,
                            double timestep = 0.5)
{
    const int goal_samples = 10;

    vector<Goal> goals;

    const double t_range = 4 * timestep;

    for (double t = T - t_range; t <= T + t_range; t += timestep)
    {
        VectorXd target_state = target.state_in(t) + delta;

        auto goal_s = target_state.head(3);
        auto goal_d = target_state.tail(3);
        goals.emplace_back(Goal{goal_s, goal_d, t});

        auto perturbed = perturb_goal(target_state, goal_samples, t);

        goals.insert(goals.end(), perturbed.begin(), perturbed.end());
    }

    return goals;
}

vector<Trajectory> generate_trajectories(const VectorXd& start_s,
                                         const VectorXd& start_d,
                                         const vector<Goal>& goals)
{
    vector<Trajectory> trajectories;
    for (const Goal& goal : goals)
    {
        auto s_coefficients = JMT(start_s, goal.s, goal.t);
        auto d_coefficients = JMT(start_d, goal.d, goal.t);
        trajectories.emplace_back(Trajectory{s_coefficients, d_coefficients, goal.t});
    }
    return trajectories;
}

std::vector<WeightedCostFunction> cost_functions = {{time_diff_cost, 0.0},
                                                    {s_diff_cost, 1.0},
                                                    {d_diff_cost, 1.0},
                                                    {collision_cost, 5.0},
                                                    {buffer_cost, 3.0}
                                                   };

Trajectory PTG(const VectorXd& start_s,
               const VectorXd& start_d,
               const Vehicle& target,
               const VectorXd& delta,
               double T,
               const vector<Vehicle>& vehicles)
{
    auto goals = generate_goals(target, delta, T);
    auto trajectories = generate_trajectories(start_s, start_d, goals);

    // Calculate trajectory costs.
    vector<TrajectoryCost> costs;
    for (const auto& trajectory : trajectories)
    {
        double cost = trajectory.calculate_cost(target, delta, T, vehicles, cost_functions);
        costs.emplace_back(TrajectoryCost{trajectory, cost});
    }

    // Find trajectory with minimum cost.
    auto it = min_element(costs.begin(), costs.end());
    return (*it).trajectory;
}
