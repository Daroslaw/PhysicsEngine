#pragma once
#include "PhysicsShape.h"
#include "PhysicsSettings.h"

class physBody;

struct physCollision
{
	physBody *A;
	physBody *B;
	physVec2 normal;
	physVec2 contacts[2];
	float penetration;
	int contactCnt;
	float restitution;
	float staticFriction;
	float dynamicFriction;

	physCollision()
	{
		this->A = nullptr;
		this->B = nullptr;
		normal = 0;
		penetration = 0;
		contactCnt = 0;
		restitution = 0;
		staticFriction = 0;
		dynamicFriction = 0;
	}

	physCollision(physBody *A, physBody *B)
	{
		if (A > B)
		{
			this->A = B;
			this->B = A;
		}
		else
		{
			this->A = A;
			this->B = B;
		}
		normal = 0;
		penetration = 0;
		contactCnt = 0;
		restitution = 0;
		staticFriction = 0;
		dynamicFriction = 0;
	}

	void Initialize();
	void Solve() const;
	void PositionalCorrection() const;
};

struct physCollisionBuffer
{
    uint32_t count;
    physCollision collisions[MAX_COLLISIONS];

    void AppendCollision(physBody * a, physBody * b);
    void ResolveAll();
    void Reset();
    void ResetHard();
};

void CircleToCircle(physCollision *col, physBody *A, physBody *B);
void CircleToPoly(physCollision *col, physBody *A, physBody *B);
void PolyToCircle(physCollision *col, physBody *A, physBody *B);
void PolyToPoly(physCollision *col, physBody *A, physBody *B);