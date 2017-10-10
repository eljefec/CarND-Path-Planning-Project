#include "planner.h"
#include "plan.h"

Planner::Planner(const Map& map)
  : map(map),
    ref_vel(0)
{
}

Path Planner::plan_path(const Telemetry& tel)
{
    Plan plan(map, tel, ref_vel);

    Path path = plan.make_plan();

    ref_vel = plan.get_ref_vel();

    return path;
}
