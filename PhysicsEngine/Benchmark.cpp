#include "Benchmark.h"

void Benchmark::RunTimer(const std::string & name)
{
    m_startedCounters[name] = 0;
    LARGE_INTEGER counterValue;
    QueryPerformanceCounter(&counterValue);
    m_startedCounters[name] = counterValue.QuadPart;
}

void Benchmark::StopTimer(const std::string & name)
{
    LARGE_INTEGER currentCounterValue;
    QueryPerformanceCounter(&currentCounterValue);
    auto startedCounterEntry = m_startedCounters.find(name);
    if (startedCounterEntry == m_startedCounters.end())
        return;
    auto counterDifference = currentCounterValue.QuadPart - startedCounterEntry->second;
    m_measuredTimes[name] = Result(counterDifference);
}

Result Benchmark::PopResult(const std::string& name)
{
    auto resultEntry = m_measuredTimes.find(name);
    if (resultEntry == m_measuredTimes.end())
        return Result();
    auto resultValue = resultEntry->second;
    m_measuredTimes.erase(name);
    return resultValue;
}
