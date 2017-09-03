#ifndef PERSPECTIVE_H
#define PERSPECTIVE_H

#include <vector>

struct Point
{
    double x;
    double y;
};

class Perspective
{
public:
    Perspective(double ref_x, double ref_y, double ref_yaw);

    void transform_to_car(std::vector<double>& ptsx,
                          std::vector<double>& ptsy) const;

    Point transform_to_car(double x, double y) const;

    void transform_to_global(std::vector<double>& ptsx,
                             std::vector<double>& ptsy) const;

private:
    double ref_x;
    double ref_y;
    double ref_yaw;
};

#endif
