#pragma once
#include <unordered_map>
#include <string>
#include "Windows.h"

#undef min
#undef max

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

    double micro() const
    {
        return (count * 1000000.0) / freq();
    }

    double milli() const
    {
        return micro() * 0.001;
    }

    double seconds() const
    {
        return milli() * 0.001;
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

    void RegisterValue(const std::string &name, unsigned long long value);
    unsigned long long PopValue(const std::string &name);

private:
    Benchmark() {}
    std::unordered_map<std::string, unsigned long long> m_startedCounters;
    std::unordered_map<std::string, Result> m_measuredTimes;
    std::unordered_map<std::string, unsigned long long> m_registeredValues;
};