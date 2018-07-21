#pragma once

#include "PhysicsBody.h"
#include "PhysicsCollision.h"

#include "BroadPhaseUniformGrid.h"
#include "BroadPhaseHierarchicalGrid.h"
#include "BroadPhaseQuadTree.h"

//  TODO:   Make some abstract class to cover all methods.

void NaiveNbyN(physBodyBufferSpan & bodies, physCollisionBuffer & collisions);

void UniformGrid(physBodyBufferSpan & bodies, physCollisionBuffer & collisions);

void HierarchicalGrid(physBodyBufferSpan & bodies, physCollisionBuffer & collisions);

void QuadTree(physBodyBufferSpan & bodies, physCollisionBuffer & collisions);