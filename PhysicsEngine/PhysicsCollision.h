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

    bool operator==(const physCollision & rhs) const
    {
        return this->A == rhs.A && this->B == rhs.B;
    }

    bool operator!=(const physCollision & rhs) const
    {
        return !(*this == rhs);
    }

    bool IsValid() const;
	void Initialize();
	void Solve() const;
	void PositionalCorrection() const;
};

struct physCollisionBuffer
{
    uint32_t rawCount;
    physCollision rawCollisions[MAX_COLLISIONS];

    uint32_t filteredCount;
    physCollision filteredCollisions[MAX_COLLISIONS];

    void AppendCollision(physBody * a, physBody * b)
    {
        if (rawCount >= MAX_COLLISIONS)
            return;

        rawCollisions[rawCount] = physCollision(a, b);
        ++rawCount;
    }
    void FilterCollisions();
    void ResolveAll();
    void Reset();
};

void CircleToCircle(physCollision *col, physBody *A, physBody *B);
void CircleToPoly(physCollision *col, physBody *A, physBody *B);
void PolyToCircle(physCollision *col, physBody *A, physBody *B);
void PolyToPoly(physCollision *col, physBody *A, physBody *B);