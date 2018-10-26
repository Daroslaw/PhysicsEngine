#pragma once
#include "PhysicsBody.h"
#include <vector>
#include <algorithm>
#include "PhysicsCollision.h"

constexpr uint8_t   HIERARCHICAL_GRID_LEVELS = 4;
constexpr uint16_t  HIERARCHICAL_GRID_NUM_OF_BUCKETS = 4001;
constexpr float     HIERARCHICAL_GRID_MIN_CELL_SIZE = 10.f;

struct GridObject
{
    GridObject() :
        body(nullptr),
        bucketIdx(UINT32_MAX),
        gridLevel(UINT8_MAX) {}

    GridObject(physBody *body) :
        body(body),
        bucketIdx(UINT32_MAX),
        gridLevel(UINT8_MAX) {}

    bool operator==(GridObject & rhs) const
    {
        return  this->body == rhs.body && 
                this->bucketIdx == rhs.bucketIdx && 
                this->gridLevel == rhs.gridLevel;
    }

    bool operator!=(GridObject & rhs) const
    {
        return !(*this == rhs);
    }

    physBody *body;
    uint32_t bucketIdx;
    uint8_t gridLevel;
};

class HierarchicalGrid
{
public:
    HierarchicalGrid()
    {
        m_buckets.resize(HIERARCHICAL_GRID_NUM_OF_BUCKETS); 
    }

    void AddObject(GridObject & newObject)
    {
        auto bodyAABB = newObject.body->GetAABB();
        auto fullExtents = bodyAABB.GetExtents() * 2;
        float longerEdge = std::max(fullExtents.x, fullExtents.y);
        float cellSize = HIERARCHICAL_GRID_MIN_CELL_SIZE;

        uint8_t level;
        for (level = 0; cellSize * 0.5f < longerEdge &&
            level < HIERARCHICAL_GRID_LEVELS - 1; ++level)
            cellSize *= 2.f;
    
        auto invCellSize = 1.f / cellSize;

        int16_t leftmostExtent = int16_t(bodyAABB.topLeft.x * invCellSize);
        int16_t rightmostExtent = int16_t(bodyAABB.botRight.x * invCellSize);
        int16_t topmostExtent = int16_t(bodyAABB.topLeft.y * invCellSize);
        int16_t bottommostExtent = int16_t(bodyAABB.botRight.y * invCellSize);

        for (auto j = leftmostExtent; j <= rightmostExtent; ++j)
            for (auto k = topmostExtent; k <= bottommostExtent; ++k)
            {
                uint32_t bucketIndex = ComputeBucketIndex(j, k, level);

                m_buckets[bucketIndex].push_back(&newObject);
                m_numOfObjectsAtLevel[level]++;
                m_occupiedLevelMask |= 1 << level;
            }

    }

    void RemoveObject(GridObject & object)
    {
        if (--m_numOfObjectsAtLevel[object.gridLevel] == 0)
            m_occupiedLevelMask &= ~(1 << object.gridLevel);
        auto bucketIdx = object.bucketIdx;
        auto& bucket = m_buckets[bucketIdx];
        bucket.erase(std::remove(bucket.begin(), bucket.end(), &object));
    }

    void SolveForObject(GridObject object, physCollisionBuffer & collisions)
    {
        float size = HIERARCHICAL_GRID_MIN_CELL_SIZE;
        uint32_t startLevel = 0;
        uint32_t maskCopy = m_occupiedLevelMask;

        auto objAABB = object.body->GetAABB();
        m_tick++;

        for(uint8_t level = startLevel;; 
            size *= 2.f, 
            maskCopy >>= 1, 
            ++level)
        {
            if (maskCopy == 0)
                break;
            if ((maskCopy & 1) == 0)
                continue;

            level = std::min(level, uint8_t(HIERARCHICAL_GRID_LEVELS - 1));

            float delta = 0;// size * HIERARCHICAL_GRID_OBJ_TO_CELL_RATIO;   //  TODO:   Needed?
            float invCellSize = 1.f / size;

            int16_t leftmostExtent = int16_t((objAABB.topLeft.x - delta) * invCellSize);
            int16_t rightmostExtent = int16_t((objAABB.botRight.x + delta) * invCellSize);
            int16_t topmostExtent = int16_t((objAABB.topLeft.y - delta) * invCellSize);
            int16_t bottommostExtent = int16_t((objAABB.botRight.y + delta) * invCellSize);

            for (auto i = leftmostExtent; i <= rightmostExtent; ++i)
                for (auto j = topmostExtent; j <= bottommostExtent; ++j)
                {
                    auto bucket = ComputeBucketIndex(i, j, level);
                    if (m_timeStamps[bucket] == m_tick)
                        continue;
                    m_timeStamps[bucket] = m_tick;

                    for(auto& otherObj : m_buckets[bucket])
                    {
                        if (*otherObj == object)
                            continue;
                        auto otherObjAABB = otherObj->body->GetAABB();
                        if (objAABB.Intersects(otherObjAABB))
                        {
                            collisions.AppendCollision(object.body, otherObj->body);
                            ++m_collisionCount;
                        }
                    }
                }
        }
    }

    void Clear()
    {
        for (auto& bucket : m_buckets)
            bucket.clear();
        m_occupiedLevelMask = 0;
        std::memset(m_numOfObjectsAtLevel, 0, sizeof m_numOfObjectsAtLevel);
        std::memset(m_timeStamps, 0, sizeof m_timeStamps);
        m_tick = 0;
        m_collisionCount = 0;
    }

    auto GetCollisionCount()
    {
        return m_collisionCount;
    }

private:

    uint32_t ComputeBucketIndex(uint32_t x, uint32_t y, uint32_t level) const
    {
        constexpr uint32_t primeNumberX = 3751102111;
        constexpr uint32_t primeNumberY = 2364347677;
        constexpr uint32_t primeNumberL = 4223902003;

        return (primeNumberX * x + primeNumberY * y + primeNumberL * level) % HIERARCHICAL_GRID_NUM_OF_BUCKETS;
    }

    uint32_t m_occupiedLevelMask = 0;
    uint32_t m_numOfObjectsAtLevel[HIERARCHICAL_GRID_LEVELS] = { 0 };
    std::vector<std::vector<GridObject*>> m_buckets;
    uint32_t m_timeStamps[HIERARCHICAL_GRID_NUM_OF_BUCKETS] = { 0 };
    uint32_t m_tick = 0;
    unsigned long long m_collisionCount = 0;
}; 

class BroadPhaseHierarchicalGrid
{
public:
    BroadPhaseHierarchicalGrid() : m_bodiesCount(0)
    {
        m_wrappedBodies.resize(MAX_BODIES);
    }

    ~BroadPhaseHierarchicalGrid()
    {

    }

    void SetBodies(physBodyBufferSpan & bodies)
    {
        m_bodiesCount = bodies.size;
        for(uint32_t i = 0; i < m_bodiesCount; ++i)
        {
            m_wrappedBodies[i] = GridObject(&bodies[i]);
            m_hg.AddObject(m_wrappedBodies[i]);
        }
    }

    void Solve(physCollisionBuffer & collisions)
    {
        for(uint32_t i = 0; i < m_bodiesCount; ++i)
        {
            m_hg.SolveForObject(m_wrappedBodies[i], collisions);
        }
        Benchmark::Get().RegisterValue("TestCount", m_hg.GetCollisionCount());
        m_hg.Clear();
    }
private:

    HierarchicalGrid m_hg;
    std::vector<GridObject> m_wrappedBodies;
    uint32_t m_bodiesCount;
};