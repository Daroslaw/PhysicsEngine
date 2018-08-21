#pragma once
#include "PhysicsBody.h"
#include "PhysicsCollision.h"

struct Bound
{
    Bound() :
        owner(nullptr),
        value(nullptr)
    {}
    Bound(physBody * owner, float * value) :
        owner(owner),
        value(value)
    {}
    physBody *owner;
    float *value;
};

class BroadPhaseSweepAndPrune
{
public:
    BroadPhaseSweepAndPrune()
    {
        
    }

    void SetBodies(physBodyBufferSpan & bodies)
    {
        m_bodyCount = bodies.size;
        for (uint32_t i = 0; i < m_bodyCount; ++i)
        {
            auto aabb = bodies[i].GetAABB();
            m_boundsX[i * 2]        = Bound(&bodies[i], &aabb.topLeft.x);
            m_boundsX[i * 2 + 1]    = Bound(&bodies[i], &aabb.botRight.x);
            m_boundsY[i * 2]        = Bound(&bodies[i], &aabb.topLeft.y);
            m_boundsY[i * 2 + 1]    = Bound(&bodies[i], &aabb.botRight.y);
        }
    }

    void Solve(physCollisionBuffer & collisions)
    {
        
    }

private:

    void SortBounds()
    {
        for(uint32_t i = 1; i < m_bodyCount * 2; ++i)
        {
            for(uint32_t j = i; j > 0; --j)
            {
                if (*m_boundsX[j - 1].value < *m_boundsX[j].value)
                    break;
                std::swap(m_boundsX[j - 1], m_boundsX[j]);
            }
        }
    }

    Bound m_boundsX[MAX_BODIES * 2];
    Bound m_boundsY[MAX_BODIES * 2];
    uint32_t m_bodyCount;
};
