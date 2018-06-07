#include "PhysicsBroadPhase.h"
#include "PhysicsSettings.h"
#include "Benchmark.h"

void NaiveNbyN(physBodyBuffer & bodies, physCollisionBuffer & collisions)
{
    for (uint32_t i = 0; i < bodies.count - 1; ++i)
    {
        for (uint32_t j = i + 1; j < bodies.count; ++j)
        {
            auto &b1 = bodies.bodies[i];
            auto &b2 = bodies.bodies[j];
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
