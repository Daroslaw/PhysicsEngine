#include "PhysicsBody.h"
#include <algorithm>

physBody::physBody()
{
	m_static = false;
	m_density = 1.f;
	m_rotation = 0.f;
	m_angularVel = 0.f;
	m_torque = 0.f;
	m_restitution = 0.5f;
	m_staticFriction = 0.3f;
	m_dynamicFriction = 0.2f;
	m_shape = nullptr;
	m_mass = 0;
	m_invMass = 0;
	m_I = 0;
	m_invI = 0;
}

void physBody::InitializePoly(physPolyShape * polyShape)
{
	m_shape = polyShape;
	m_shape->Initialize();
	CalculateMassData();
}

void physBody::InitializeCircle(physCircleShape * circleShape)
{
	m_shape = circleShape;
	m_shape->Initialize();
	CalculateMassData();
}

void physBody::Refresh()
{
	m_force = 0;
	m_torque = 0;
}

void physBody::ApplyForce(const physVec2 & force, const physVec2& contactVector)
{
	if (m_static)
		return;
	m_force += force;
	m_torque += force.cross(contactVector);
}

void physBody::ApplyImpulse(const physVec2 & impulse, const physVec2 & contactVector)
{
	if (m_static)
		return;
	m_linearVel += impulse * m_invMass;
	m_angularVel += contactVector.cross(impulse) * m_invI;
}

void physBody::SetStatic(bool flag)
{
	if (flag)
	{
		m_mass = 0;
		m_invMass = 0;
		m_I = 0;
		m_invI = 0;
	}
	else
		CalculateMassData();
	m_static = flag;
}

bool physBody::IsStatic()
{
	return m_static;
}

void physBody::SetPosition(const physVec2 & pos)
{
	//m_transform.pos = pos;
	m_position = pos;
}

void physBody::SetRotation(float angle)
{
	//m_transform.rot.set(angle);
	//m_rotation.set(angle);
	m_rotation.set(angle);
}

void physBody::SetDensity(float density)
{
	m_density = density;
	CalculateMassData();
}

void physBody::SetRestitution(float restitution)
{
	m_restitution = (abs(restitution) < 1) ? abs(restitution) : 1;
}

void physBody::SetStaticFriction(float stFriction)
{
	m_staticFriction = (abs(stFriction) < 1) ? abs(stFriction) : 1;
}

void physBody::SetDynamicFriction(float dnFriction)
{
	m_dynamicFriction = (abs(dnFriction) < 1) ? abs(dnFriction) : 1;
}

float physBody::GetDensity() const
{
	return m_density;
}

float physBody::GetRestitution() const
{
	return m_restitution;
}

float physBody::GetStaticFriction() const
{
	return m_staticFriction;
}

float physBody::GetDynamicFriction() const
{
	return m_dynamicFriction;
}

physShape * physBody::GetShape() const
{
	return m_shape;
}

physAABB physBody::GetAABB() const
{
	return m_aabb;
}

physVec2 physBody::GetPos() const
{
	return m_position;
}

physRot physBody::GetRot() const
{
	return m_rotation;
}

physVec2 physBody::GetLinearVelocity()
{
	return m_linearVel;
}

float physBody::GetAngularVelocity()
{
	return m_angularVel;
}

float physBody::GetMass()
{
	return m_mass;
}

float physBody::GetInvMass()
{
	return m_invMass;
}

float physBody::GetI()
{
	return m_I;
}

float physBody::GetInvI()
{
	return m_invI;
}

void physBody::CalculateMassData()
{
	m_mass = 0.f;
	m_I = 0.f;

	m_mass = m_density * m_shape->GetArea();
	m_invMass = 1.f / m_mass;

	// http://mathoverflow.net/questions/73556/calculating-moment-of-inertia-in-2d-planar-polygon

	switch (m_shape->GetType())
	{
		case physShape::sCircle:
		{
			physCircleShape *c = dynamic_cast<physCircleShape*>(m_shape);
			m_I = static_cast<float>(M_PI_2) * powf(c->GetRadius(), 4);
			break;
		}
		case physShape::sPoly:
		{
			physPolyShape *p = dynamic_cast<physPolyShape*>(m_shape);
			physPolyHandle &ph = p->GetPolygon();
			for (unsigned int i = 0; i < ph.GetCount() - 1; ++i)
			{
				physVec2 v1 = ph.At(i).position;
				physVec2 v2 = ph.At(i + 1).position;

				m_I += (powf(v1.x, 2) + powf(v1.y, 2) + (v1.x * v2.x) + (v1.y * v2.y) + powf(v2.x, 2) + powf(v2.y, 2)) * v1.cross(v2);
				//m_I += ((v1.x * v2.y) + (2 * v1.x * v1.y) + (2 * v2.x * v2.y) + (v2.x * v1.y)) * v1.cross(v2);
			}
			m_I *= m_density / 12.f;
			m_I = (m_I < 0) ? abs(m_I) : m_I;
			break;
		}
		default:
		{
			m_I = 1.f;
			break;
		}
	}

	m_invI = 1.f / m_I;
}

void physBody::CalculateAABB()
{
	if (GetShape()->GetType() == physShape::Type::sCircle)
	{
		physCircleShape *cs = dynamic_cast<physCircleShape*>(GetShape());
		float rad = cs->GetRadius();
		m_aabb = physAABB(m_position + physVec2(-rad, -rad), m_position + physVec2(rad, rad));
	}
	else if (GetShape()->GetType() == physShape::Type::sPoly)
	{
		physPolyShape *ps = dynamic_cast<physPolyShape*>(GetShape());

		auto ph = ps->GetPolygon();

		float left = FLT_MAX;
		float top = FLT_MAX;
		float right = -FLT_MAX;
		float bottom = -FLT_MAX;

		auto rot = GetRot();

		for (uint32_t i = 0; i < ph.GetCount(); ++i)
		{
			physVec2 vec = ph.At(i).position;
			vec.rotate(rot);

			left = std::min(left, vec.x);
			right = std::max(right, vec.x);
			top = std::min(top, vec.y);
			bottom = std::max(bottom, vec.y);
		}

		m_aabb = physAABB(m_position + physVec2(left, top), m_position + physVec2(right, bottom));
	}
}

void physBodyBuffer::Reset()
{
    count = 0;
}

void physBodyBuffer::ResetHard()
{
    for (uint32_t i = 0; i < count; ++i)
        bodies[i] = physBody();
    Reset();
}
