#include "PhysicsBroadPhase.h"
#include "PhysicsSettings.h"
#include "Benchmark.h"

void NaiveNbyN(physBodyBufferSpan & bodies, physCollisionBuffer & collisions)
{
    for (uint32_t i = 0; i < bodies.size - 1; ++i)
    {
        for (uint32_t j = i + 1; j < bodies.size; ++j)
        {
            auto &b1 = bodies[i];
            auto &b2 = bodies[j];
            if (&b1 == &b2)
                continue;
            auto &aabb1 = b1.GetAABB();
            auto &aabb2 = b2.GetAABB();
            if (aabb1.Intersects(aabb2))
                collisions.AppendCollision(&b1, &b2);
        }
    }
}

template<class BuildingStrategy, class PartitioningStrategy, class TraversalStrategy>
void BoundingVolumeHierarchy(physBodyBuffer & bodies, physCollisionBuffer & collisions)
{

}
