#ifndef UTIL_H
#define UTIL_H

#include <cmath>

double distance(double x1, double y1, double x2, double y2);

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);

#endif
