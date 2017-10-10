#ifndef PLAN_H
#define PLAN_H

#include <memory>
#include <vector>

#include "environment.h"
#include "map.h"
#include "telemetry.h"
#include "trajectory.h"
#include "vehicle.h"

struct Path
{
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
};

struct PrevTail
{
    double ref_x;
    double ref_y;
    double ref_yaw;
};

struct Waypoints
{
    std::vector<double> ptsx;
    std::vector<double> ptsy;
};

class Plan
{
public:
    Plan(const Map& map, const Telemetry& tel, double ref_vel);

    Path make_plan();

    double get_ref_vel() const;

private:
    // Static helpers.
    static void print(const std::vector<double>& ptsx,
                      const std::vector<double>& ptsy,
                      const char* func);

    static bool has_spline_violation(const std::vector<double>& ptsx);

    static bool check_spline_has_violation(const std::vector<double>& ptsx,
                                           const std::vector<double>& ptsy,
                                           const char* func);

    // Private helpers.
    void generate_waypoints();

    PrevTail fill_prev_tail(Waypoints& waypoints);

    Trajectory pass_or_follow(const PrevTail& prev_tail,
                              const Waypoints& waypoints);

    bool allow_lane_change();

    void get_trajectory_points(const Trajectory& trajectory,
                               std::vector<double>& ptsx,
                               std::vector<double>& ptsy);

    void keep_lane_with_spline(Waypoints& waypoints);

    bool make_smooth_path(const PrevTail& prevtail,
                          const Trajectory* p_trajectory,
                          const Waypoints& waypoints,
                          Path& path);
    // Members
    const Map& map;
    const Telemetry& tel;
    const Environment env;

    const int car_lane;
    const int prev_size;

    double ref_vel;

    std::unique_ptr<Vehicle> forward_vehicle;

    Path path;
};

#endif
