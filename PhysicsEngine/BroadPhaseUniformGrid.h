#pragma once

#include <vector>
#include <algorithm>

#include "PhysicsBody.h"
#include "PhysicsCollision.h"
#include "Benchmark.h"

constexpr uint32_t OPEN_HASHING_TABLE_SIZE = 4001;
constexpr uint32_t CLOSED_HASHING_TABLE_SIZE = 8009;
constexpr uint32_t FIXED_GRID_CELL_SIZE = 25.f;

#define PROBING_LINEAR 1
#define PROBING_QUAD 2
#define PROBING_DHASH 4
#define PROBING PROBING_QUAD

struct IGrid
{
    virtual ~IGrid(){}
    virtual void MapObjects(physBodyBufferSpan bodies) = 0;
    virtual void Solve(physCollisionBuffer & collisions) = 0;
    virtual void Clear() = 0;
    virtual int GetSize() = 0;
};

struct FixedGrid : IGrid
{
    FixedGrid() {}
    FixedGrid(const physVec2 & position, uint16_t width, uint16_t height, float cellSize) :
        topLeftCorner(position),
        width(width),
        height(height),
        cellSize(cellSize),
        invCellSize(1.f/cellSize)
    {
        buckets.resize(width * height);
        //  TODO:   Reserve or not?
        for (auto bucket : buckets)
            bucket.reserve(8);
    }

    void MapObjects(physBodyBufferSpan bodies) override
    {
        for (uint16_t i = 0; i < bodies.size; ++i)
        {
            auto body = &bodies[i];
            auto& aabb = body->GetAABB();

            int16_t leftmostExtent = int16_t((aabb.topLeft.x - topLeftCorner.x) * invCellSize);
            int16_t rightmostExtent = int16_t((aabb.botRight.x - topLeftCorner.x) * invCellSize);
            int16_t topmostExtent = int16_t((aabb.topLeft.y - topLeftCorner.y) * invCellSize);
            int16_t bottommostExtent = int16_t((aabb.botRight.y - topLeftCorner.y) * invCellSize);

            constexpr int16_t zero = 0;
            leftmostExtent = std::max(leftmostExtent, zero);
            rightmostExtent = std::min(rightmostExtent, int16_t(width - 1));
            topmostExtent = std::max(topmostExtent, zero);
            bottommostExtent = std::min(bottommostExtent, int16_t(height - 1));

            for (auto j = leftmostExtent; j <= rightmostExtent; ++j)
                for (auto k = topmostExtent; k <= bottommostExtent; ++k)
                    buckets[k * width + j].push_back(body);
        }
        
    }

    void Solve(physCollisionBuffer & collisions) override
    {
        unsigned long long testCount = 0;
        for (auto& bucket : buckets)
        {
            for (int i = 0; i < int(bucket.size() - 1); ++i)
            {
                for (int j = i + 1; j < bucket.size(); ++j)
                {
                    auto b1 = bucket[i];
                    auto b2 = bucket[j];
                    auto &aabb1 = b1->GetAABB();
                    auto &aabb2 = b2->GetAABB();
                    if (aabb1.Intersects(aabb2))
                        collisions.AppendCollision(b1, b2);
                    ++testCount;
                }
            }
        }
        Benchmark::Get().RegisterValue("TestCount", testCount);
    }

    void Clear() override
    {
        for (auto& bucket : buckets)
            bucket.clear();
    }

    int GetSize() override
    {
        unsigned size = 0;
        for (auto& bucket : buckets)
            size += bucket.size();
        return size;
    }

    physVec2 topLeftCorner;
    uint16_t width = 0;
    uint16_t height = 0;
    float cellSize = 0.f;
    float invCellSize = 0.f;

    std::vector<std::vector<physBody*>> buckets;
};

struct OpenHashGrid : IGrid
{
    OpenHashGrid(const physVec2 & position, uint32_t numOfBuckets, float cellSize) :
        topLeftCorner(position),
        numOfBuckets(numOfBuckets),
        cellSize(cellSize),
        invCellSize(1.f/cellSize)
    {
        buckets.resize(numOfBuckets);
        //  TODO:   Reserve or not?
        for (auto bucket : buckets)
            bucket.reserve(8);
    }

    void MapObjects(physBodyBufferSpan bodies) override
    {
        for (uint16_t i = 0; i < bodies.size; ++i)
        {
            auto body = &bodies[i];
            auto& aabb = body->GetAABB();

            int16_t leftmostExtent = int16_t((aabb.topLeft.x - topLeftCorner.x) * invCellSize);
            int16_t rightmostExtent = int16_t((aabb.botRight.x - topLeftCorner.x) * invCellSize);
            int16_t topmostExtent = int16_t((aabb.topLeft.y - topLeftCorner.y) * invCellSize);
            int16_t bottommostExtent = int16_t((aabb.botRight.y - topLeftCorner.y) * invCellSize);

            for (auto j = leftmostExtent; j <= rightmostExtent; ++j)
                for (auto k = topmostExtent; k <= bottommostExtent; ++k)
                {
                    constexpr uint32_t randomPrimeX = 3233651589;
                    constexpr uint32_t randomPrimeY = 1955760773;
                    uint32_t bucketIndex = (j * randomPrimeX + k * randomPrimeY) % numOfBuckets;
                    buckets[bucketIndex].push_back(body);
                }
        }
    }

    void Solve(physCollisionBuffer & collisions)  override
    {
        unsigned long long testCount = 0;

        for (auto& bucket : buckets)
        {
            for (int i = 0; i < int(bucket.size() - 1); ++i)
            {
                for (int j = i + 1; j < bucket.size(); ++j)
                {
                    auto b1 = bucket[i];
                    auto b2 = bucket[j];
                    auto &aabb1 = b1->GetAABB();
                    auto &aabb2 = b2->GetAABB();
                    if (aabb1.Intersects(aabb2))
                        collisions.AppendCollision(b1, b2);
                    ++testCount;
                }
            }
        }

        Benchmark::Get().RegisterValue("TestCount", testCount);
    }

    void Clear() override
    {
        for (auto& bucket : buckets)
            bucket.clear();
    }

    int GetSize() override
    {
        unsigned size = 0;
        for (auto& bucket : buckets)
            size += bucket.size();
        return size;
    }

    physVec2 topLeftCorner;
    uint32_t numOfBuckets;
    float cellSize = 0.f;
    float invCellSize = 0.f;

    std::vector<std::vector<physBody*>> buckets;
};

struct ClosedHashGrid : IGrid
{
    ClosedHashGrid(const physVec2 & position, uint32_t tableSize, float cellSize) :
        topLeftCorner(position),
        tableSize(tableSize),
        cellSize(cellSize),
        invCellSize(1.f/cellSize)
    {
        hashTable = new physBody*[tableSize];
        ClosedHashGrid::Clear();
    }

    void MapObjects(physBodyBufferSpan bodies) override
    {
        for (uint16_t i = 0; i < bodies.size; ++i)
        {
            auto body = &bodies[i];
            auto& aabb = body->GetAABB();

            int16_t leftmostExtent = int16_t((aabb.topLeft.x - topLeftCorner.x) * invCellSize);
            int16_t rightmostExtent = int16_t((aabb.botRight.x - topLeftCorner.x) * invCellSize);
            int16_t topmostExtent = int16_t((aabb.topLeft.y - topLeftCorner.y) * invCellSize);
            int16_t bottommostExtent = int16_t((aabb.botRight.y - topLeftCorner.y) * invCellSize);

            minLeftmostExtent = std::min(leftmostExtent, minLeftmostExtent);
            maxRightmostExtent = std::max(rightmostExtent, maxRightmostExtent);
            minTopmostExtent = std::min(topmostExtent, minTopmostExtent);
            maxBottommostExtent = std::max(bottommostExtent, maxBottommostExtent);

            for (auto j = leftmostExtent; j <= rightmostExtent; ++j)
                for (auto k = topmostExtent; k <= bottommostExtent; ++k)
                {
                    uint32_t index = ComputeIndex(j, k);
#if (PROBING == PROBING_DHASH)
                    uint32_t indexBase = index;
                    uint32_t index2 = Probing(index);
#endif
                    uint32_t iterations = 0;
                    while (hashTable[index] != nullptr)
                    {
                        if (hashTable[index] == body)
                            break;
#if (PROBING == PROBING_LINEAR) || (PROBING == PROBING_QUAD)
                        index = Probing(index, ++iterations);
#elif (PROBING == PROBING_DHASH)
                        index = (indexBase + (++iterations) * index2) % tableSize;
#endif
                    }
                    hashTable[index] = body;
                }
        }
    }

    void Solve(physCollisionBuffer & collisions) override
    {
        unsigned long long testCount = 0;
        for(int i = minLeftmostExtent; i <= maxRightmostExtent; ++i)
        {
            for(int j = minTopmostExtent; j <= maxBottommostExtent; ++j)
            {
                uint32_t index = ComputeIndex(i, j);
#if (PROBING == PROBING_DHASH)
                uint32_t indexBase = index;
                uint32_t index2 = Probing(index);
#endif
                auto body = hashTable[index];
                if (body == nullptr)
                    continue;
                std::vector<physBody*> currentBucket;
                currentBucket.reserve(8);
                uint32_t iterations = 0;
                while(body != nullptr)
                {
                    for (auto bodyFromBucket : currentBucket)
                    {
                        auto &aabb1 = body->GetAABB();
                        auto &aabb2 = bodyFromBucket->GetAABB();
                        if (aabb1.Intersects(aabb2))
                            collisions.AppendCollision(bodyFromBucket, body);
                        ++testCount;
                    }
                    currentBucket.push_back(body);
#if (PROBING == PROBING_LINEAR) || (PROBING == PROBING_QUAD)
                    index = Probing(index, ++iterations);
#elif (PROBING == PROBING_DHASH)
                    index = (indexBase + (++iterations) * index2) % tableSize;
#endif
                    body = hashTable[index];
                }
            }
        }
        Benchmark::Get().RegisterValue("TestCount", testCount);
    }

    void Clear() override
    {
        for(uint32_t i = 0; i < tableSize; ++i)
            hashTable[i] = nullptr;

        minLeftmostExtent = INT16_MAX;
        maxRightmostExtent = INT16_MIN;
        minTopmostExtent = INT16_MAX;
        maxBottommostExtent = INT16_MIN;
    }

    int GetSize() override
    {
        unsigned size = 0;
        for (int i = 0; i < tableSize; ++i)
            size += 1 * (hashTable[i] != nullptr);
        return size;
    }

    uint32_t ComputeIndex(int16_t x, int16_t y) const
    {
        constexpr uint32_t randomPrimeX = 3233651589;
        constexpr uint32_t randomPrimeY = 1955760773;

        return (x * randomPrimeX + y * randomPrimeY) % tableSize;
    }
    
    uint32_t Probing(uint32_t oldIndex, uint32_t state = 0) const
    {
        uint32_t index;
#if (PROBING == PROBING_LINEAR)
        constexpr int16_t interval = 1;
        index = (oldIndex + interval) % tableSize;
#elif (PROBING == PROBING_QUAD)
        index = (oldIndex + state * state) % tableSize;
#elif (PROBING == PROBING_DHASH)
        constexpr uint32_t randomPrime = 2003;
        index = randomPrime - (oldIndex % randomPrime);
#endif
        return index;
    }

    int16_t minLeftmostExtent;
    int16_t maxRightmostExtent;
    int16_t minTopmostExtent;
    int16_t maxBottommostExtent;

    physVec2 topLeftCorner;
    uint32_t tableSize;
    float cellSize = 0.f;
    float invCellSize = 0.f;

    physBody** hashTable;
};

class BroadPhaseUniformGrid
{
public:
    explicit BroadPhaseUniformGrid(physBodyBufferSpan & bodies, const physVec2 & topLeftCornerPosition, const physVec2 & sizeExtents) : 
        m_bodies(bodies)
    {
        auto cellSize = CalculateCellSize();
#if (BP == BP_UNIFORM_FIXED) || (BP == BP_UNIFORM)
        auto width = uint16_t(sizeExtents.x / cellSize) + 1;
        auto height = uint16_t(sizeExtents.y / cellSize) + 1;
        m_grid = new FixedGrid(topLeftCornerPosition, width, height, cellSize);
#elif BP == BP_UNIFORM_OPENHASH
        m_grid = new OpenHashGrid(topLeftCornerPosition, OPEN_HASHING_TABLE_SIZE, cellSize);
#elif BP == BP_UNIFORM_CLOSEDHASH
        m_grid = new ClosedHashGrid(topLeftCornerPosition, CLOSED_HASHING_TABLE_SIZE, cellSize);
#endif
    }
    ~BroadPhaseUniformGrid()
    {
        delete m_grid;
    }

    void SetBodies(physBodyBufferSpan & bodies);
    void Solve(physCollisionBuffer & collisions) const;

private:

    float CalculateCellSize() const;

    physBodyBufferSpan m_bodies;
    IGrid *m_grid;
};

inline void BroadPhaseUniformGrid::SetBodies(physBodyBufferSpan & bodies)
{
    m_bodies = bodies;
}

inline void BroadPhaseUniformGrid::Solve(physCollisionBuffer& collisions) const
{
    m_grid->MapObjects(m_bodies);
    Benchmark::Get().RegisterValue("Size", m_grid->GetSize());
    m_grid->Solve(collisions);
    m_grid->Clear();
}


//  TODO:   Research more strategies concerning cell size calculations
inline float BroadPhaseUniformGrid::CalculateCellSize() const
{
#if 0
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
#elif 0
    //  Average Object Strategy
    float sumOfExtents = 0;
    for (uint16_t i = 0; i < m_bodies.size; ++i)
    {
        auto& aabb = m_bodies[i].GetAABB();
        auto extents = aabb.GetExtents() * 2;
        sumOfExtents += std::max(extents.x, extents.y);
    }
    return sumOfExtents / m_bodies.size;
#else 
    //  Fixed size
    return FIXED_GRID_CELL_SIZE;
#endif
}
