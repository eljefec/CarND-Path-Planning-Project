#include "stopwatch.h"

#include <iostream>

using namespace std;
using namespace std::chrono;

Stopwatch::Stopwatch(bool print_at_destruction)
  : print_at_destruction(print_at_destruction),
    start(high_resolution_clock::now())
{
}

Stopwatch::~Stopwatch()
{
    if (print_at_destruction)
    {
        cout << elapsed_time<microseconds>() << " microsecs" << endl;
    }
}
