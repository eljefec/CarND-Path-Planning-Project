#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <memory>
#include <vector>
#include "vehicle.h"

class Environment
{
public:
    Environment(const std::vector<double>& maps_s,
                const std::vector<double>& maps_x,
                const std::vector<double>& maps_y,
                double car_s,
                double car_d,
                const std::vector<std::vector<double>>& sensor_fusion,
                size_t prev_size);

    std::unique_ptr<Vehicle> lane_is_occupied(int lane);

    std::vector<double> getFrenet(double x, double y, double theta);

    std::vector<double> getXY(double s, double d);

private:
    const std::vector<double>& maps_s;
    const std::vector<double>& maps_x;
    const std::vector<double>& maps_y;
    double car_s;
    double car_d;
    std::vector<Vehicle> vehicles;
    size_t prev_size;
};

#endif
