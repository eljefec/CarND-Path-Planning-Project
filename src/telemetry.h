#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <vector>
#include "json.hpp"

struct Telemetry
{
    Telemetry(nlohmann::json& telemetry);

    // Main car's localization Data
    const double car_x;
    const double car_y;
    const double car_s;
    const double car_d;
    const double car_yaw;
    const double car_speed;

    // Previous path data given to the Planner
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;

    // Previous path's end s and d values
    const double end_path_s;
    const double end_path_d;

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    std::vector<std::vector<double>> sensor_fusion;
};

#endif
