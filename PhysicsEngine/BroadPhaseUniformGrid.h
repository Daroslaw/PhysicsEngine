#pragma once

#include <vector>
#include <algorithm>

#include "PhysicsBody.h"
#include "PhysicsCollision.h"

struct FixedGrid
{
    physVec2 topLeftCorner;
    uint16_t width = 0;
    uint16_t height = 0;
    float cellSize = 0.f;
    float invCellSize = 0.f;

    FixedGrid() {}
    FixedGrid(const physVec2 & position, uint16_t width, uint16_t height, float cellSize) :
        topLeftCorner(position),
        width(width),
        height(height),
        cellSize(cellSize),
        invCellSize(1/cellSize)
    {
        buckets.resize(width * height);
        //  TODO:   Reserve or not?
        for (auto bucket : buckets)
            bucket.reserve(8);
    }

    void MapObject(physBody * body)
    {
        auto& aabb = body->GetAABB();
        
        uint16_t leftmostExtent = uint16_t(aabb.topLeft.x * invCellSize);
        uint16_t rightmostExtent = uint16_t(aabb.botRight.x * invCellSize);
        uint16_t topmostExtent = uint16_t(aabb.topLeft.y * invCellSize);
        uint16_t bottommostExtent = uint16_t(aabb.botRight.y * invCellSize);

        constexpr uint16_t zero = 0;
        leftmostExtent = std::max(leftmostExtent, zero);
        rightmostExtent = std::min(rightmostExtent, uint16_t(width - 1));
        topmostExtent = std::max(topmostExtent, zero);
        bottommostExtent = std::min(bottommostExtent, uint16_t(height - 1));

        for(auto i = leftmostExtent; i <= rightmostExtent; ++i)
            for(auto j = topmostExtent; j <= bottommostExtent; ++j)
                buckets[j * width + i].push_back(body);
    }

    void Clear()
    {
        for (auto& bucket : buckets)
            bucket.clear();
    }

    std::vector<std::vector<physBody*>> buckets;
};

class BroadPhaseUniformGrid
{
public:
    explicit BroadPhaseUniformGrid(physBodyBuffer & bodies, const physVec2 & topLeftCornerPosition, const physVec2 & sizeExtents) : 
        m_bodies(bodies.AsSpan())
    {
        auto cellSize = CalculateCellSize();
        auto width = uint16_t(sizeExtents.x / cellSize + 1);
        auto height = uint16_t(sizeExtents.y / cellSize + 1);
        m_grid = FixedGrid(topLeftCornerPosition, width, height, cellSize);
    }

    void SetBodies(physBodyBuffer & bodies);
    void Solve(physCollisionBuffer & collisions);

private:

    float CalculateCellSize() const;
    void AssignObjectsToBuckets();

    physBodyBufferSpan m_bodies;
    FixedGrid m_grid;
};

inline void BroadPhaseUniformGrid::SetBodies(physBodyBuffer & bodies)
{
    m_bodies = bodies.AsSpan();
}

inline void BroadPhaseUniformGrid::Solve(physCollisionBuffer& collisions)
{
    AssignObjectsToBuckets();

    for(auto& bucket : m_grid.buckets)
    {
        for(int i = 0; i < int(bucket.size() - 1); ++i)
        {
            for(int j = i + 1; j < bucket.size(); ++j)
            {
                auto b1 = bucket[i];
                auto b2 = bucket[j];
                auto &aabb1 = b1->GetAABB();
                auto &aabb2 = b2->GetAABB();
                if (aabb1.Intersects(aabb2))
                    collisions.AppendCollision(b1, b2);
            }
        }
    }
    m_grid.Clear();
}

inline float BroadPhaseUniformGrid::CalculateCellSize() const
{
    //  Largest Object Strategy
    constexpr auto SQRT2 = float(M_SQRT2);
    float largestExtent = 0;
    for(uint16_t i = 0; i < m_bodies.size; ++i)
    {
        auto& aabb = m_bodies[i].GetAABB();
        auto extents = aabb.GetExtents() * 2;
        largestExtent = std::max(largestExtent, std::max(extents.x, extents.y));
    }
    return largestExtent * SQRT2;
}

inline void BroadPhaseUniformGrid::AssignObjectsToBuckets()
{
    for (uint16_t i = 0; i < m_bodies.size; ++i)
        m_grid.MapObject(&m_bodies[i]);
}
