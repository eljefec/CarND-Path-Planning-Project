#include "stopwatch.h"

#include <iostream>

using namespace std;
using namespace std::chrono;

static bool enable_printing = true;

Stopwatch::Stopwatch(const char* name)
  : name(name),
    start(high_resolution_clock::now())
{
}

Stopwatch::~Stopwatch()
{
    if (enable_printing && (name != nullptr))
    {
        cout << name << ": " << elapsed_time<microseconds>() << " microsecs" << endl;
    }
}
