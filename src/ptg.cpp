#include <algorithm>
#include <random>
#include <unordered_map>
#include <utility>
#include "constants.h"
#include "cost_functions.h"
#include "jmt.h"
#include "ptg.h"
#include "stopwatch.h"

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

struct PTG_Goal
{
    Vehicle target;
    VectorXd delta;
    double T;
};

vector<Goal> generate_goals(const PTG_Goal& ptg_goal,
                            const int goal_samples_per_timestep,
                            double timestep = 0.5)
{
    Stopwatch sw(__func__);

    vector<Goal> goals;

    const double t_range = 2 * timestep;

    for (double t = ptg_goal.T - t_range; t <= ptg_goal.T + t_range; t += timestep)
    {
        VectorXd target_state = ptg_goal.target.state_in(t) + ptg_goal.delta;

        auto goal_s = target_state.head(3);
        auto goal_d = target_state.tail(3);
        goals.emplace_back(Goal{goal_s, goal_d, t});

        auto perturbed = perturb_goal(target_state, goal_samples_per_timestep, t);

        goals.insert(goals.end(), perturbed.begin(), perturbed.end());
    }

    return goals;
}

vector<Trajectory> generate_trajectories(const VectorXd& start_s,
                                         const VectorXd& start_d,
                                         const vector<Goal>& goals,
                                         const unordered_map<double, JMT>& time_jmt_map)
{
    Stopwatch sw(__func__);

    vector<Trajectory> trajectories;
    for (const Goal& goal : goals)
    {
        const auto it = time_jmt_map.find(goal.t);
        if (it == time_jmt_map.end())
        {
            throw runtime_error("Failed to find JMT for goal.");
        }
        else
        {
            const JMT& jmt = it->second;
            auto s_coefficients = jmt.solve(start_s, goal.s);
            auto d_coefficients = jmt.solve(start_d, goal.d);
            trajectories.emplace_back(Trajectory{s_coefficients, d_coefficients, goal.t});
        }
    }
    return trajectories;
}

unordered_map<double, JMT> generate_jmts(const vector<Goal>& goals)
{
    Stopwatch sw(__func__);

    unordered_map<double, JMT> time_jmt_map;

    for (const auto& goal : goals)
    {
        if (time_jmt_map.find(goal.t) == time_jmt_map.end())
        {
            time_jmt_map.insert(make_pair(goal.t, JMT(goal.t)));
        }
    }

    return time_jmt_map;
}

Trajectory PTG(const VectorXd& start_s,
               const VectorXd& start_d,
               const vector<PTG_Goal>& ptg_goals,
               const vector<Vehicle>& vehicles)
{
    Stopwatch sw(__func__);

    vector<TrajectoryCost> costs;

    const int goal_samples_per_timestep = 10 / ptg_goals.size();

    for (const PTG_Goal& ptg_goal : ptg_goals)
    {
        auto goals = generate_goals(ptg_goal, goal_samples_per_timestep);
        auto jmts = generate_jmts(goals);
        auto trajectories = generate_trajectories(start_s, start_d, goals, jmts);

        Stopwatch sw("Calc trajectory costs");

        // Calculate trajectory costs.
        for (const auto& trajectory : trajectories)
        {
            CostFunctions cost_functions(start_s, start_d, trajectory, ptg_goal.target, ptg_goal.delta, ptg_goal.T, vehicles);
            double cost = cost_functions.cost();
            // cout << "cost:" << cost << endl;
            costs.emplace_back(TrajectoryCost{trajectory, cost});
        }
    }

    // Find trajectory with minimum cost.
    auto it = min_element(costs.begin(), costs.end());
    // cout << "best cost:" << it->cost << endl;
    return (*it).trajectory;
}
