#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "telemetry.h"

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
};

class Planner
{
public:
    Planner(const Map& map);

    Path plan_path(const Telemetry& telemetry);

private:
    const Map map;

    double ref_vel; // mph

};

#endif
