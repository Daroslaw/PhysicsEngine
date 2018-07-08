#pragma once

#include "PhysicsBody.h"
#include "PhysicsCollision.h"

#include "BroadPhaseUniformGrid.h"

void NaiveNbyN(physBodyBufferSpan & bodies, physCollisionBuffer & collisions);

void UniformGrid(physBodyBufferSpan & bodies, physCollisionBuffer & collisions);

template <class BuildingStrategy, class PartitioningStrategy, class TraversalStrategy>
void BoundingVolumeHierarchy(physBodyBuffer & bodies, physCollisionBuffer & collisions);

