#include "util.h"
#include <cmath>

using namespace std;

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double deg2rad(double x)
{
    return x * pi() / 180;
}

double rad2deg(double x)
{
    return x * 180 / pi();
}
