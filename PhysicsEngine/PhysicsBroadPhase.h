#pragma once

#include "PhysicsBody.h"
#include "PhysicsCollision.h"


#include "BroadPhaseUniformGrid.h"
#include "BroadPhaseHierarchicalGrid.h"

void NaiveNbyN(physBodyBufferSpan & bodies, physCollisionBuffer & collisions);

void UniformGrid(physBodyBufferSpan & bodies, physCollisionBuffer & collisions);

void HierarchicalGrid(physBodyBufferSpan & bodies, physCollisionBuffer & collisions);

