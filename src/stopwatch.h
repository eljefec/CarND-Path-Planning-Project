#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <chrono>

class Stopwatch
{
public:
    Stopwatch(bool print_at_destruction);

    ~Stopwatch();

    template<typename DurationType>
    long int elapsed_time() const
    {
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<DurationType>(end - start).count();
    }

private:
    bool print_at_destruction;
    std::chrono::high_resolution_clock::time_point start;
};

#endif
