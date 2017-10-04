#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <chrono>

class Stopwatch
{
public:
    // When name is not null, the destructor of Stopwatch will print its elapsed time.
    // Use __func__ for name to print the function's name.
    Stopwatch(const char* name = nullptr);

    ~Stopwatch();

    template<typename DurationType>
    long int elapsed_time() const
    {
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<DurationType>(end - start).count();
    }

private:
    const char* name;
    std::chrono::high_resolution_clock::time_point start;
};

#endif
