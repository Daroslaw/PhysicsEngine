#pragma once
#include <unordered_map>
#include <string>
#include "Windows.h"

static unsigned long long freq()
{
    LARGE_INTEGER frequency;
    QueryPerformanceFrequency(&frequency);
    return frequency.QuadPart;
}

struct Result
{
    unsigned long long count;

    Result()
    {
        count = 0;
    }
    Result(unsigned long long count)
    {
        this->count = count;
    }

    double seconds() const
    {
        return count * 1.0 / freq();
    }

    double milli() const
    {
        return seconds() * 1000.;
    }

    double micro() const
    {
        return milli() * 1000.;
    }
};

class Benchmark
{
public:
    static Benchmark &Get()
    {
        static Benchmark instance;
        return instance;
    }
    Benchmark(Benchmark const&) = delete;
    void operator=(Benchmark const&) = delete;

    void RunTimer(const std::string& name);
    void StopTimer(const std::string& name);
    Result PopResult(const std::string& name);

private:
    Benchmark() {}
    std::unordered_map<std::string, unsigned long long> m_startedCounters;
    std::unordered_map<std::string, Result> m_measuredTimes;
};