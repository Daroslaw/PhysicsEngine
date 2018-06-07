#pragma once

#include "PhysicsBody.h"
#include "PhysicsCollision.h"
#include "PhysicsSettings.h"

class physWorld
{
public:
	physWorld();

	void SetGravity(const physVec2& gravity);

	void Simulate(float dt = DT);

	physCollisionBuffer *GetCollisions();

	physBody* RegisterPoly(float x, float y, physVec2 *vertices, unsigned int vertCount);
	physBody* RegisterBox(float x, float y, float a);
	physBody* RegisterRectangle(float x, float y, float w, float h);
	physBody* RegisterCircle(float x, float y, float r);

	void DestroyAll();
	void DestroyBody(physBody *body) const;

private:	
	void InitializeBody(physBody *body) const;
	void ClearBody(physBody *body) const;
	void UpdateVelocity(physBody *body, float dt) const;
	void UpdatePosition(physBody *body, float dt) const;

	void BroadPhase();
	void NarrowPhase();
	void ResolveCollisions();

	physVec2 m_gravity;

    physBodyBuffer m_bodies;

	unsigned int m_curCircleIdx;
	physCircleShape m_allCircles[MAX_CIRCLES];

	unsigned int m_curPolyIdx;
	physPolyShape m_allPolys[MAX_POLYGONS];

	physCollisionBuffer m_collisions;

	physPolyHandler m_polyHandler;
};
