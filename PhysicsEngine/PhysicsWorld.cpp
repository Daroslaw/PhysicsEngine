#include "PhysicsWorld.h"
#include "PhysicsBroadPhase.h"
#include <algorithm>
#include "Benchmark.h"

physWorld::physWorld()
{
	m_curCircleIdx = 0;
	m_curPolyIdx = 0;
    m_bodies.Reset();
    m_collisions.Reset();
}

void physWorld::SetGravity(const physVec2 & gravity)
{
	m_gravity = gravity;
}

void physWorld::Simulate(float dt)
{
	for (unsigned int i = 0; i < m_bodies.count; ++i)
	{
		auto & curBody = m_bodies.bodies[i];

		InitializeBody(&curBody);
	}

	GetCollisions()->Reset();
	BroadPhase();
    GetCollisions()->FilterCollisions();
	NarrowPhase();
	ResolveCollisions();

	for (unsigned int i = 0; i < m_bodies.count; ++i)
	{
		auto & curBody = m_bodies.bodies[i];
		UpdateVelocity(&curBody, dt);
		UpdatePosition(&curBody, dt);
		ClearBody(&curBody);
	}
}

physCollisionBuffer * physWorld::GetCollisions()
{
	return &m_collisions;
}

physBody * physWorld::RegisterPoly(float x, float y, physVec2 * vertices, unsigned int vertCount)
{
	if (m_bodies.count >= MAX_BODIES || m_curPolyIdx >= MAX_POLYGONS)
		return nullptr;

	physBody &curBody = m_bodies.bodies[m_bodies.count];
	physPolyShape &curPoly = m_allPolys[m_curPolyIdx];

	auto polyHandle = m_polyHandler.CreatePoly(vertices, vertCount);
	if (polyHandle.GetCount() == 1)
		return nullptr;

	curPoly = physPolyShape(polyHandle);
	curBody.InitializePoly(&m_allPolys[m_curPolyIdx]);
	curBody.SetPosition({ x, y });

	++m_curPolyIdx;
	++m_bodies.count;

	return &curBody;
}

physBody * physWorld::RegisterBox(float x, float y, float a)
{
	return RegisterRectangle(x, y, a, a);
}

physBody * physWorld::RegisterRectangle(float x, float y, float w, float h)
{
	physVec2 edges[4];
	float hw = w / 2;
	float hh = h / 2;

	edges[0] = { -hw, -hh };
	edges[1] = { hw, -hh };
	edges[2] = { hw, hh };
	edges[3] = { -hw, hh };

	return RegisterPoly(x, y, edges, 4);
}

physBody * physWorld::RegisterCircle(float x, float y, float r)
{
	if (m_bodies.count >= MAX_BODIES || m_curCircleIdx >= MAX_CIRCLES)
		return nullptr;

	physBody &curBody = m_bodies.bodies[m_bodies.count];
	physCircleShape &curCircle = m_allCircles[m_curCircleIdx];

	curCircle = physCircleShape(r);
	curBody.InitializeCircle(&curCircle);
	curBody.SetPosition({ x, y });

	++m_bodies.count;
	++m_curCircleIdx;

	return &curBody;
}

void physWorld::DestroyAll()
{
	for (uint32_t i = 0; i < m_curCircleIdx; ++i)
		m_allCircles[i] = physCircleShape();
	for (uint32_t i = 0; i < m_curPolyIdx; ++i)
		m_allPolys[i] = physPolyShape();

	m_curCircleIdx = 0;
	m_curPolyIdx = 0;

    m_bodies.ResetHard();
    m_collisions.Reset();
	m_polyHandler.DestroyAll();
}

void physWorld::DestroyBody(physBody * body) const
{
	//	TODO
}

void physWorld::InitializeBody(physBody * body) const
{
	body->CalculateAABB();
}

void physWorld::ClearBody(physBody* body) const
{
	body->Refresh();
}

void physWorld::UpdateVelocity(physBody * body, float dt) const
{
	if (body == nullptr) return;
	if (body->IsStatic()) return;

	body->m_linearVel += (body->m_force * body->m_invMass + m_gravity) * dt *0.5f;
	body->m_angularVel += body->m_torque * body->m_invI * dt *0.5f;
}

void physWorld::UpdatePosition(physBody * body, float dt) const
{
	if (body == nullptr) return;
	if (body->IsStatic()) return;

	body->m_position += body->m_linearVel * dt;
	body->m_rotation.rotateBy(body->m_angularVel * dt);
	UpdateVelocity(body, dt);
}

void physWorld::BroadPhase()
{
    Benchmark::Get().RunTimer("Collision");
    auto bodySpan = m_bodies.AsSpan();
#if BP == BP_NAIVE
    NaiveNbyN(bodySpan, m_collisions);
#elif (BP & BP_UNIFORM) != 0
    UniformGrid(bodySpan, m_collisions);
#elif BP == BP_HIERARCHICAL_GRID
    HierarchicalGrid(bodySpan, m_collisions);
#elif BP == BP_QUADTREE
    QuadTree(bodySpan, m_collisions);
#endif
    Benchmark::Get().StopTimer("Collision");
}

void physWorld::NarrowPhase()
{
    if (m_collisions.filteredCount == 0)
        return;
    for (uint32_t i = 0; i < MAX_COLLISIONS; ++i)
    {
        auto &col = m_collisions.filteredCollisions[i];
        if (!col.IsValid())
            continue;
        auto b1 = col.A;
		auto b2 = col.B;
		auto b1type = b1->GetShape()->GetType();
		auto b2type = b2->GetShape()->GetType();

		if (b1type == physShape::Type::sCircle)
		{
			if (b2type == physShape::Type::sCircle)
			{
				CircleToCircle(&col, b1, b2);
			}
			else if (b2type == physShape::Type::sPoly)
			{
				CircleToPoly(&col, b1, b2);
			}
		}
		else if (b1type == physShape::Type::sPoly)
		{
			if (b2type == physShape::Type::sCircle)
			{
				PolyToCircle(&col, b1, b2);

			}
			else if (b2type == physShape::Type::sPoly)
			{
				PolyToPoly(&col, b1, b2);

			}
		}
	}
}

void physWorld::ResolveCollisions()
{
    m_collisions.ResolveAll();
}
