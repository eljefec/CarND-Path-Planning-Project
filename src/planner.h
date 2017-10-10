#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "map.h"
#include "plan.h"
#include "telemetry.h"

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
