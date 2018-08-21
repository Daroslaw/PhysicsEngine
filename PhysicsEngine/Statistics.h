#pragma once
#include <vector>
#include <numeric>

class MovingAverage
{
public:
    MovingAverage(uint16_t bufferSize)
    {
        m_buffer.resize(bufferSize);
        m_init = true;
    }
    double Get(double nextValue)
    {
        static uint16_t index = 0;

        if (!m_init)
            m_buffer[index++ % m_buffer.size()] = nextValue;
        else
        {
            m_buffer.assign(m_buffer.size(), nextValue);
            m_init = false;
        }
        
        return accumulate(m_buffer.begin(), m_buffer.end(), 0.0) / m_buffer.size();
    }
private:
    bool m_init;
    std::vector<double> m_buffer;
};