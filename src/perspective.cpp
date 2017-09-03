#include "perspective.h"
#include <cmath>

Perspective::Perspective(double ref_x, double ref_y, double ref_yaw)
  : ref_x(ref_x),
    ref_y(ref_y),
    ref_yaw(ref_yaw)
{
}

void Perspective::transform_to_car(std::vector<double>& ptsx,
                                   std::vector<double>& ptsy) const
{
    for (int i = 0; i < ptsx.size(); i++)
    {
        Point p = transform_to_car(ptsx[i], ptsy[i]);
        ptsx[i] = p.x;
        ptsy[i] = p.y;
    }
}

Point Perspective::transform_to_car(double x, double y) const
{
    double shift_x = x - ref_x;
    double shift_y = y - ref_y;

    double result_x = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    double result_y = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

    return Point{result_x, result_y};
}

void Perspective::transform_to_global(std::vector<double>& ptsx,
                                      std::vector<double>& ptsy) const
{
    for (int i = 0; i < ptsx.size(); i++)
    {
        double x = ptsx[i];
        double y = ptsy[i];

        double x_point = (x * cos(ref_yaw) - y * sin(ref_yaw));
        double y_point = (x * sin(ref_yaw) + y * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        ptsx[i] = x_point;
        ptsy[i] = y_point;
    }
}
