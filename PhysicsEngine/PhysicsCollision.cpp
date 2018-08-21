#include "PhysicsCollision.h"
#include "PhysicsBody.h"
#include <vector>
#include "Benchmark.h"

bool physCollision::IsValid() const
{
    return A != nullptr && B != nullptr;
}

void physCollision::Initialize()
{
	if (!IsValid())
		return;
	if (A->IsStatic() && B->IsStatic())
		return;
	
	restitution = std::min(A->GetRestitution(), B->GetRestitution());
	staticFriction = sqrtf(A->GetStaticFriction() * B->GetStaticFriction());
	dynamicFriction = sqrtf(A->GetDynamicFriction() * B->GetDynamicFriction());
}

void physCollision::Solve() const
{
	if (A->IsStatic() && B->IsStatic())
		return;

	physVec2 impulse;
	physVec2 tanImpulse;
	for (int i = 0; i < contactCnt; ++i)
	{
		physVec2 radA = contacts[i] - A->GetPos();
		physVec2 radB = contacts[i] - B->GetPos();

		physVec2 relativeVel =	B->GetLinearVelocity() + radB.cross(-B->GetAngularVelocity()) -
								A->GetLinearVelocity() - radA.cross(-A->GetAngularVelocity());
		float contactVel = relativeVel.dot(normal);
		if (contactVel > 0)
			return;

		float rAxN = radA.cross(normal);
		float rBxN = radB.cross(normal);

		float invMassSum = A->GetInvMass() + B->GetInvMass();
		invMassSum += rAxN * rAxN * A->GetInvI();
		invMassSum += rBxN * rBxN * B->GetInvI();

		float j = -(1.0f + restitution) * contactVel;
		j /= invMassSum;
		j /= contactCnt;

		//if (impulse.lengthSquared() == 0)
		impulse = normal * j;
		A->ApplyImpulse(-impulse, radA);
		B->ApplyImpulse(impulse, radB);

		//	TODO: frictions

		physVec2 t = relativeVel - (normal * relativeVel.dot(normal));
		t.normalize();

		float jt = relativeVel.dot(t);
		jt /= invMassSum;
		jt /= contactCnt;

		if (abs(jt) < 0.001)
			return;

		if (tanImpulse.lengthSquared() == 0)
		{
			if (abs(jt) < abs(j * staticFriction))
				tanImpulse = t * -jt;
			else
				tanImpulse = t * -j * dynamicFriction;
		}

		A->ApplyImpulse(-tanImpulse, radA);
		B->ApplyImpulse(tanImpulse, radB);
	}
}

void physCollision::PositionalCorrection() const
{
	if (A->IsStatic() && B->IsStatic())
		return;

	constexpr float k = 0.01f;
	constexpr float p = 0.1f;
	float coeff = std::max(penetration - k, 0.f) / (A->GetInvMass() + B->GetInvMass());
	if (coeff == 0)
		return;
	physVec2 corr = -normal * coeff * p;
	if (!A->IsStatic())
		A->SetPosition(A->GetPos() + (corr * A->GetInvMass()));
	if (!B->IsStatic())
		B->SetPosition(B->GetPos() - (corr * B->GetInvMass()));
}

void physCollisionBuffer::FilterCollisions()
{
    for(int i = 0; i < rawCount; ++i)
    {
        physCollision & rawCollision = rawCollisions[i];
        constexpr uint32_t primeNumberA = 1231872409;
        constexpr uint32_t primeNumberB = 3116752669;
        uint32_t index = (primeNumberA * uint32_t(rawCollision.A) + 
                          primeNumberB * uint32_t(rawCollision.B)) % MAX_COLLISIONS;

        while (filteredCollisions[index].IsValid() && filteredCollisions[index] != rawCollision)
            index = (index + 1) % MAX_COLLISIONS;

        if (filteredCollisions[index] == rawCollision)
            continue;

        filteredCollisions[index] = rawCollision;
        ++filteredCount;
    }
}

void physCollisionBuffer::ResolveAll()
{

    for (uint32_t i = 0; i < MAX_COLLISIONS; ++i)
    {
        auto& col = filteredCollisions[i];
        if (!col.IsValid())
            continue;
        col.Initialize();
        col.Solve();
        col.PositionalCorrection();
    }

    Benchmark::Get().RegisterValue("AllCollisions", rawCount);
    Benchmark::Get().RegisterValue("UniqueCollisions", filteredCount);
}

void physCollisionBuffer::Reset()
{
    for (uint32_t i = 0; i < MAX_COLLISIONS; ++i)
        filteredCollisions[i] = physCollision();
    rawCount = 0;
    filteredCount = 0;
}

void CircleToCircle(physCollision * col, physBody * A, physBody * B)
{
	physCircleShape *As = dynamic_cast<physCircleShape*>(A->GetShape());
	physCircleShape *Bs = dynamic_cast<physCircleShape*>(B->GetShape());

	physVec2 normal = B->GetPos() - A->GetPos();
	float distanceSqrd = normal.lengthSquared();
	float radSum = As->GetRadius() + Bs->GetRadius();

	if (distanceSqrd > radSum * radSum)
		return;

	float distance = sqrtf(distanceSqrd);
	col->contactCnt = 1;

	if (distance == 0.f)
	{
		col->penetration = As->GetRadius();
		col->normal = physVec2(1, 0);
		col->contacts[0] = A->GetPos();
	}
	else
	{
		col->penetration = radSum - distance;
		col->normal = normal / distance;
		col->contacts[0] = col->normal * As->GetRadius() + A->GetPos();
	}
}

void CircleToPoly(physCollision * col, physBody * A, physBody * B)
{
	physCircleShape *As = dynamic_cast<physCircleShape*>(A->GetShape());
	physPolyShape *Bs = dynamic_cast<physPolyShape*>(B->GetShape());

	float rad = As->GetRadius();
	physRot polyRot = B->GetRot();
	physVec2 center = A->GetPos() - B->GetPos();
	center.rotate(-polyRot);

	float separation = -FLT_MAX;
	int faceNormal = 0;

	auto &ph = Bs->GetPolygon();
	for (uint32_t i = 0; i < ph.GetCount(); ++i)
	{
		float s = ph.At(i).normal.dot(center - ph.At(i).position);

		if (s > rad)
			return;

		if (s > separation)
		{
			separation = s;
			faceNormal = i;
		}
	}

	physVec2 faceV1 = ph.At(faceNormal).position;
	physVec2 faceV2 = ph.At(faceNormal + 1).position;

	if (separation < 0.0001)
	{
		col->contactCnt = 1;
		col->normal = ph.At(faceNormal).normal;
		col->normal.rotate(polyRot);
		col->normal = -col->normal;
		col->contacts[0] = col->normal * rad + A->GetPos();
		col->penetration = rad;
		return;
	}

	float d1 = (center - faceV1).dot(faceV2 - faceV1);
	float d2 = (center - faceV2).dot(faceV1 - faceV2);
	col->penetration = rad - separation;

	if (d1 <= 0.f)
	{
		if ((center - faceV1).lengthSquared() > rad * rad)
			return;

		physVec2 n = faceV1 - center;
		n.rotate(polyRot);
		n.normalize();
		faceV1.rotate(polyRot);
		col->contactCnt = 1;
		col->normal = n;
		col->contacts[0] = faceV1 + B->GetPos();
	}
	else if (d2 <= 0.f)
	{
		if ((center - faceV2).lengthSquared() > rad * rad)
			return;

		physVec2 n = faceV2 - center;
		n.rotate(polyRot);
		n.normalize();
		faceV2.rotate(polyRot);
		col->contactCnt = 1;
		col->normal = n;
		col->contacts[0] = faceV2 + B->GetPos();
	}
	else
	{
		physVec2 n = ph.At(faceNormal).normal;
		if ((center - faceV1).dot(n) > rad)
			return;
		n.rotate(polyRot);
		col->contactCnt = 1;
		col->normal = -n;
		col->contacts[0] = col->normal * rad + A->GetPos();
	}
}

void PolyToCircle(physCollision * col, physBody * A, physBody * B)
{
	CircleToPoly(col, B, A);
	col->normal = -col->normal;
}

static physVec2 gjkGetFarthestPntInDir(physBody * body, const physVec2 d)
{
	auto polyShape = dynamic_cast<physPolyShape*>(body->GetShape());
	auto rot = body->GetRot();
	auto &ph = polyShape->GetPolygon();

	physVec2 result;
	float maxDot = -FLT_MIN;
	for (uint32_t i = 0; i < ph.GetCount(); ++i)
	{
		physVec2 curVec = ph.At(i).position;
		curVec.rotate(rot);
		float curDot = curVec.dot(d);
		if(curDot > maxDot)
		{
			maxDot = curDot;
			result = curVec;
		}
	}
	return result + body->GetPos();
}

static physVec2 gjkGetSupportPoint(physBody * b1, physBody * b2, const physVec2 d)
{
	physVec2 p1 = gjkGetFarthestPntInDir(b1, d);
	physVec2 p2 = gjkGetFarthestPntInDir(b2, -d);

	physVec2 minkowskiDifference = p1 - p2;
	return minkowskiDifference;
}

static bool gjkSimplexContainsOrigin(std::vector<physVec2> &simplex, physVec2 &d)
{
	if (simplex.size() < 2)
		return false;

	physVec2 a = simplex.back();
	
	physVec2 ao = -a;

	if(simplex.size() == 3)
	{
		physVec2 b = simplex.at(1);
		physVec2 c = simplex.at(0);

		physVec2 ab = b - a;
		physVec2 ac = c - a;

		physVec2 abPerp = tripleProduct(ac, ab, ab);
		physVec2 acPerp = tripleProduct(ab, ac, ac);

		if(abPerp.dot(ao) >= 0)
		{
			simplex.erase(simplex.begin());
			d = abPerp;
		}
		else if (acPerp.dot(ao) >= 0)
		{
			simplex.erase(simplex.begin() + 1);
			d = acPerp;
		}
		else
			return true;
	}
	else
	{
		physVec2 b = simplex.at(0);
		physVec2 ab = b - a;
		if (ab.dot(ao) >= 0)
		{
			physVec2 abPerp = tripleProduct(ab, ao, ab);
			d = abPerp;
		}
		else
			d = ao;
	}
	return false;
}

static void cpGetMostPerpendicularEdge(physBody *body, physVec2 normal, physVec2 &ve1, physVec2 &ve2, physVec2 &vMaxProj)
{
	auto polyShape = dynamic_cast<physPolyShape*>(body->GetShape());
	auto rot = body->GetRot();
	auto &ph = polyShape->GetPolygon();

	int idx = 0;
	float maxProjection = -FLT_MAX;

	for (uint32_t i = 0; i < ph.GetCount(); ++i)
	{
		physVec2 v = ph.At(i).position;
		v.rotate(rot);
		float projection = normal.dot(v);
		if(maxProjection < projection)
		{
			maxProjection = projection;
			idx = i;
		}
	}
	int idxNext = (idx + 1) % ph.GetCount();
	int idxPrev = (idx - 1 < 0) ? ph.GetCount() - 1 : idx - 1;
	physVec2 v = ph.At(idx).position;
	physVec2 vNext = ph.At(idxNext).position;
	physVec2 vPrev = ph.At(idxPrev).position;

	physVec2 left = v - vNext;
	physVec2 right = v - vPrev;
	left.rotate(rot);
	right.rotate(rot);

	left.normalize();
	right.normalize();

	if (left.dot(normal) >= right.dot(normal))
	{
		ve1 = vPrev;
		ve2 = v;
	}
	else
	{
		ve1 = v;
		ve2 = vNext;
	}

	vMaxProj = v;
	ve1.rotate(rot);
	ve2.rotate(rot);
	vMaxProj.rotate(rot);
}

static std::vector<physVec2> cpClip(physVec2 v1, physVec2 v2, physVec2 normal, float offset)
{
	std::vector<physVec2> result;
    result.reserve(3);
	float d1 = normal.dot(v1) - offset;
	float d2 = normal.dot(v2) - offset;

	if (d1 >= 0) result.push_back(v1);
	if (d2 >= 0) result.push_back(v2);

	if(d1 * d2 < 0)
	{
		physVec2 e = v2 - v1;

		float u = d1 / (d1 - d2);
		e = e * u + v1;
		result.push_back(e);
	}
	return result;
}

void PolyToPoly(physCollision * col, physBody * A, physBody * B)
{
	//	GJK
	std::vector<physVec2> simplex;
	physVec2 dir = { 0, -1 };
	physVec2 support = gjkGetSupportPoint(A, B, dir);
	simplex.push_back(support);
	dir = -support;
	while (true)
	{
		physVec2 a = gjkGetSupportPoint(A, B, dir);
		simplex.push_back(a);

		if (a.dot(dir) <= 0)
			return;
		if(gjkSimplexContainsOrigin(simplex, dir))
			break;
	}

	//	EPA
	int closestIdx = 0;
	physVec2 normal;
	float distance;;

	while(true)
	{
		distance = FLT_MAX;
		for (int i = 0; i < simplex.size(); ++i)
		{
			int j = (i + 1) % simplex.size();

			physVec2 ab = simplex[j] - simplex[i];
			physVec2 n = tripleProduct(ab, simplex[i], ab);
			n.normalize();

			float d = n.dot(simplex[i]);
			if(d < distance)
			{
				distance = d;
				normal = n;
				closestIdx = j;
			}
		}

		support = gjkGetSupportPoint(A, B, normal);
		float d = support.dot(normal);
		if (abs(d - distance) < 0.001)
		{
			col->normal = normal;
			col->penetration = d;
			break;
		}
		
		simplex.insert(simplex.begin() + closestIdx, support);
	}

	//	CLIPPING
	physVec2 relPos = B->GetPos() - A->GetPos();
	physVec2 eAv1, eAv2, eBv1, eBv2, eRmax, eImax;
	cpGetMostPerpendicularEdge(A, normal, eAv1, eAv2, eRmax);
	cpGetMostPerpendicularEdge(B, -normal, eBv1, eBv2, eImax);
	physVec2 eA = eAv2 - eAv1;
	eBv1 += relPos;
	eBv2 += relPos;
	physVec2 eB = eBv2 - eBv1;
	physVec2 eRv1, eRv2, eIv1, eIv2, ref, inc;
	bool flip = false;
	if(abs(eA.dot(normal)) <= abs(eB.dot(normal)))
	{
		ref = eA;
		eRv1 = eAv1;
		eRv2 = eAv2;
		inc = eB;
		eIv1 = eBv1;
		eIv2 = eBv2;
	}
	else
	{
		ref = eB;
		eRv1 = eBv1;
		eRv2 = eBv2;
		inc = eA;
		eIv1 = eAv1;
		eIv2 = eAv2;
		flip = true;
	}
	ref.normalize();
	std::vector<physVec2> cp;

	float offset1 = ref.dot(eRv1);
	cp = cpClip(eIv1, eIv2, ref, offset1);
	if (cp.size() < 2) return;

	float offset2 = ref.dot(eRv2);
	cp = cpClip(cp[0], cp[1], -ref, -offset2);
	if (cp.size() < 2) return;

	physVec2 refNorm = -ref.cross(1.0f);
	float max = refNorm.dot(eRmax);
	int delIdx = 1;
	if (refNorm.dot(cp[0]) - max < 0.0f)
	{
		cp.erase(cp.begin());
		delIdx = 0;
	}
	if (refNorm.dot(cp[delIdx]) - max < 0.0f)
		cp.erase(cp.begin() + delIdx);

	for (int i = 0; i < cp.size(); i++)
	{
		col->contactCnt = i + 1;
		col->contacts[i] = cp.at(i) + A->GetPos();
	}
	
}