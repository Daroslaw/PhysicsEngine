#pragma once

#include "PhysicsBody.h"
#include "PhysicsCollision.h"


void NaiveNbyN(physBodyBuffer & bodies, physCollisionBuffer & collisions);

class TreeBuilder
{
    
};

template <class BuildingStrategy, class PartitioningStrategy, class TraversalStrategy>
void BoundingVolumeHierarchy(physBodyBuffer & bodies, physCollisionBuffer & collisions);

