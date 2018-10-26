#include "PhysicsBroadPhase.h"
#include "PhysicsSettings.h"
#include "Benchmark.h"

void NaiveNbyN(physBodyBufferSpan & bodies, physCollisionBuffer & collisions)
{
    unsigned long long testCount = 0;
    for (uint32_t i = 0; i < bodies.size - 1; ++i)
    {
        for (uint32_t j = i + 1; j < bodies.size; ++j)
        {
            auto &b1 = bodies[i];
            auto &b2 = bodies[j];
            auto &aabb1 = b1.GetAABB();
            auto &aabb2 = b2.GetAABB();
            if (aabb1.Intersects(aabb2))
                collisions.AppendCollision(&b1, &b2);
            ++testCount;
        }
    }
    Benchmark::Get().RegisterValue("TestCount", testCount);
}

void UniformGrid(physBodyBufferSpan & bodies, physCollisionBuffer & collisions)
{
    static BroadPhaseUniformGrid bp(bodies, physVec2(0, 0), physVec2(800, 800));
    bp.SetBodies(bodies);
    bp.Solve(collisions);
}

void HierarchicalGrid(physBodyBufferSpan& bodies, physCollisionBuffer& collisions)
{
    static BroadPhaseHierarchicalGrid bp;
    bp.SetBodies(bodies);
    bp.Solve(collisions);
}

void QuadTree(physBodyBufferSpan& bodies, physCollisionBuffer& collisions)
{
    static BroadPhaseQuadTree bp(physVec2(400, 400), 400);
    bp.SetBodies(bodies);
    bp.Solve(collisions);
}
