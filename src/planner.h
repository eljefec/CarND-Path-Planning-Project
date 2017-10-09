#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "perspective.h"
#include "telemetry.h"
#include "trajectory.h"

struct Path
{
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
};

struct Map
{
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;
    double max_s;
};

class Planner
{
public:
    Planner(const Map& map);

    Path plan_path(const Telemetry& telemetry);

private:
    bool make_smooth_path(const Perspective& perspective,
                          const Trajectory* p_trajectory,
                          std::vector<double>& ptsx,
                          std::vector<double>& ptsy,
                          double target_x,
                          int path_size,
                          Path& path);

    const Map map;

    double ref_vel; // mph

};

#endif
