#include "statistics.h"

using namespace std;

Statistics calculate_stats(const vector<double>& values)
{
    double total = 0;
    for (auto v : values)
    {
        total += v;
    }

    double mean = total / values.size();

    total = 0;

    for (auto v : values)
    {
        double diff = v - mean;
        total += diff * diff;
    }

    double stddev = total / values.size();

    return Statistics{mean, stddev};
}
