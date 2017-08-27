#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using Eigen::VectorXd;

class Vehicle
{
public:
    Vehicle(const std::vector<double>& sensor_fusion)
      : id(sensor_fusion[0]),
        x(sensor_fusion[1]),
        y(sensor_fusion[2]),
        vx(sensor_fusion[3]),
        vy(sensor_fusion[4]),
        s(sensor_fusion[5]),
        d(sensor_fusion[6]),
        speed(sqrt(vx * vx + vy * vy))
    {
    }

    Eigen::VectorXd state_in(double t) const
    {
        // Assume vehicle is travelling straight.
        VectorXd state(6);
        state << s + speed * t,
                 speed,
                 0,
                 d,
                 0,
                 0;
    }

private:
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    double speed;
};

#endif