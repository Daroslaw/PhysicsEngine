#pragma once

#include "PhysicsShape.h"
#include "PhysicsSettings.h"

class physBody
{
	friend class physWorld;

public:
	physBody();

	void ApplyForce(const physVec2& force, const physVec2& contactVector = 0);
	void ApplyImpulse(const physVec2& impulse, const physVec2& contactVector = 0);

	void SetStatic(bool flag = true);
	bool IsStatic();

	void SetPosition(const physVec2& pos);
	void SetRotation(float angle);

	physVec2 GetLinearVelocity();
	float GetAngularVelocity();

	float GetMass();
	float GetInvMass();
	float GetI();
	float GetInvI();

	void SetDensity(float density);
	void SetRestitution(float restitution);
	void SetStaticFriction(float stFriction);
	void SetDynamicFriction(float dnFriction);

	float GetDensity() const;
	float GetRestitution() const;
	float GetStaticFriction() const;
	float GetDynamicFriction() const;

	physShape *GetShape() const;
	physAABB GetAABB() const;
	physVec2 GetPos() const;
	physRot GetRot() const;

private:

	void InitializePoly(physPolyShape *polyShape);
	void InitializeCircle(physCircleShape *circleShape);

	void Refresh();

	void CalculateMassData();
	void CalculateAABB();

	bool m_static;
	physShape *m_shape;
	physAABB m_aabb;

	physVec2 m_position;
	physRot m_rotation;

	float m_angularVel;
	float m_torque;
	physVec2 m_linearVel;
	physVec2 m_force;

	float m_mass, m_invMass;
	float m_I, m_invI;

	float m_density;
	float m_restitution;
	float m_staticFriction;
	float m_dynamicFriction;
};

struct physBodyBuffer
{
    unsigned int count;
    physBody bodies[MAX_BODIES];

    void Reset();
    void ResetHard();
};