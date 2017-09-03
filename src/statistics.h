#ifndef STATISTICS_H
#define STATISTICS_H

#include <vector>

struct Statistics
{
    double mean;
    double stddev;
};

Statistics calculate_stats(const std::vector<double>& values);

#endif
