#include "telemetry.h"

template <typename T, typename U>
void Fill(std::vector<T>& vec, const U& json)
{
    for (const auto& val : json)
    {
        vec.push_back(val);
    }
}

Telemetry::Telemetry(nlohmann::json& j)
  : car_x(j[1]["x"]),
    car_y(j[1]["y"]),
    car_s(j[1]["s"]),
    car_d(j[1]["d"]),
    car_yaw(j[1]["yaw"]),
    car_speed(j[1]["speed"]),
    end_path_s(j[1]["end_path_s"]),
    end_path_d(j[1]["end_path_d"])
{
    Fill(previous_path_x, j[1]["previous_path_x"]);
    Fill(previous_path_y, j[1]["previous_path_y"]);
    Fill(sensor_fusion, j[1]["sensor_fusion"]);
}
