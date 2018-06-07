#pragma once

#include "PhysicsPoly.h"

struct physAABB
{
	physAABB() { topLeft = 0; botRight = 0; }
	physAABB(physVec2 tl, physVec2 br) { topLeft = tl; botRight = br; }
	physVec2 topLeft;
	physVec2 botRight;

	inline physVec2 GetCenter() const;
	inline physVec2 GetExtents() const;
	inline bool Contains(const physAABB& aabb) const;
	inline bool Intersects(const physAABB& aabb) const;
	inline physAABB Combine(const physAABB& rhs) const;
};

#pragma region physAABB definitions
inline physVec2 physAABB::GetCenter() const
{
	return (topLeft + botRight) * 0.5f;
}
inline physVec2 physAABB::GetExtents() const
{
	return (botRight - topLeft) * 0.5f;
}
inline bool physAABB::Contains(const physAABB& aabb) const
{
	bool result;
	result = (this->topLeft.x <= aabb.topLeft.x) &&
		(this->topLeft.y <= aabb.topLeft.y) &&
		(this->botRight.x >= aabb.botRight.x) &&
		(this->botRight.y >= aabb.botRight.y);
	return result;
}
inline bool physAABB::Intersects(const physAABB& aabb) const
{
	if (this->topLeft.x > aabb.botRight.x) return false;
	if (this->topLeft.y > aabb.botRight.y) return false;
	if (this->botRight.x < aabb.topLeft.x) return false;
	if (this->botRight.y < aabb.topLeft.y) return false;
	return true;
}
inline physAABB physAABB::Combine(const physAABB& rhs) const
{
	float left = (this->topLeft.x > rhs.topLeft.x) ? rhs.topLeft.x : this->topLeft.x;
	float top = (this->topLeft.y > rhs.topLeft.y) ? rhs.topLeft.y : this->topLeft.y;
	float right = (this->botRight.x < rhs.botRight.x) ? rhs.botRight.x : this->botRight.x;
	float bottom = (this->botRight.y < rhs.botRight.y) ? rhs.botRight.y : this->botRight.y;

	return physAABB({left, top}, {right, bottom});
}
#pragma endregion

class physShape
{
public:
	enum Type
	{
		sCircle,
		sPoly
	};

	physShape() : m_area(0) {};
	virtual ~physShape() {};

	virtual void Initialize() = 0;

	virtual Type GetType() = 0;
	float GetArea();

protected:
	virtual void ComputeArea() = 0;
	
	float m_area;
};

class physCircleShape : public physShape
{
public:
	physCircleShape();
	explicit physCircleShape(float radius);

	void Initialize() override;

	Type GetType() override;
	float GetRadius() const;

protected:
	void ComputeArea() override;

private:
	float m_radius;
};

class physPolyShape : public physShape
{
public:
	physPolyShape();
	explicit physPolyShape(physPolyHandle polygon);

	void Initialize() override;

	Type GetType() override;
	physPolyHandle & GetPolygon();

protected:
	void ComputeArea() override;
	void AdjustByCentroid() const;

private:
	physPolyHandle m_polygon;
};