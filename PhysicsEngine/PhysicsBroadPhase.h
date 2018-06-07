#pragma once

#include "PhysicsBody.h"
#include "PhysicsCollision.h"


void NaiveNbyN(physBodyBufferSpan & bodies, physCollisionBuffer & collisions);

template <class BuildingStrategy, class PartitioningStrategy, class TraversalStrategy>
void BoundingVolumeHierarchy(physBodyBuffer & bodies, physCollisionBuffer & collisions);

