#pragma once
#include <vector>
#include <numeric>

namespace bm
{
    class MovingAverage
    {
    public:
        MovingAverage(uint16_t bufferSize)
        {
            m_buffer.resize(bufferSize);
        }
        double Get(double nextValue)
        {
            static uint16_t index = 0;
            m_buffer[index++ % m_buffer.size()] = nextValue;
            return accumulate(m_buffer.begin(), m_buffer.end(), 0.0) / m_buffer.size();
        }
    private:
        std::vector<double> m_buffer;
    };
}
