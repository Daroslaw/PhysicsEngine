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
        m_index = 0;
    }
    double Get(double nextValue)
    {
        if (!m_init)
        {
            m_buffer[m_index] = nextValue;
            m_index = (m_index + 1) % m_buffer.size();
        }
        else
        {
            m_buffer.assign(m_buffer.size(), nextValue);
            m_init = false;
        }
        
        return accumulate(m_buffer.begin(), m_buffer.end(), 0.0) / m_buffer.size();
    }
private:
    bool m_init;
    unsigned m_index;
    std::vector<double> m_buffer;
};