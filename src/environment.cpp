#include "environment.h"

using namespace std;

Environment::Environment(double car_s,
            double car_d,
            const vector<vector<double>>& sensor_fusion,
            size_t prev_size)
  : car_s(car_s),
    car_d(car_d),
    prev_size(prev_size)
{
    for (const auto& obstacle : sensor_fusion)
    {
        vehicles.emplace_back(Vehicle(obstacle));
    }
}

std::unique_ptr<Vehicle> Environment::lane_is_occupied(int lane)
{
    const float lane_width = 4;

    std::unique_ptr<Vehicle> forward_vehicle;

    for (const auto& v : vehicles)
    {
        float d = v.d;
        if (d < (lane_width * lane + lane_width) && d > (lane_width * lane))
        {
            double vx = v.vx;
            double vy = v.vy;
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = v.s;

            for (size_t i = 0; i < prev_size; i++)
            {
                check_car_s += 0.02 * check_speed;

                if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
                {
                    if (forward_vehicle)
                    {
                        if (v.s < forward_vehicle->s)
                        {
                            forward_vehicle.reset(new Vehicle(v));
                        }
                    }
                    else
                    {
                        forward_vehicle.reset(new Vehicle(v));
                    }
                }
            }
        }
    }

    return forward_vehicle;
}
