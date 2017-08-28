#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <memory>
#include "vehicle.h"

class Environment
{
public:
    Environment(double car_s,
                double car_d,
                const std::vector<std::vector<double>>& sensor_fusion,
                size_t prev_size);

    std::unique_ptr<Vehicle> lane_is_occupied(int lane);

private:
    double car_s;
    double car_d;
    std::vector<Vehicle> vehicles;
    size_t prev_size;
};

#endif
