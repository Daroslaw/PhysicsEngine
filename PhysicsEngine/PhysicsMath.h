#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>

struct physRot;

struct physVec2
{
	float x;
	float y;

	physVec2(float x = 0, float y = 0);

	inline float lengthSquared() const;
	inline float length() const;
	inline float dot(const physVec2& rhs) const;
	inline float cross(const physVec2& rhs) const;
	inline physVec2 cross(float scalar) const;
	inline void normalize();
	inline void rotate(const physRot& rot);

	inline physVec2 operator+(const physVec2& rhs) const;
	inline physVec2 operator-(const physVec2& rhs) const;
	inline physVec2 operator-() const;
	inline physVec2& operator+=(const physVec2& rhs);
	inline physVec2& operator-=(const physVec2& rhs);
	inline bool operator<(const physVec2& rhs) const;
	inline bool operator>(const physVec2& rhs) const;
	inline bool operator<=(const physVec2& rhs) const;
	inline bool operator>=(const physVec2& rhs) const;
	inline physVec2 operator*(const float scalar) const;
	inline physVec2 operator/(const float scalar) const;	
};

struct physRot
{
	float sin;
	float cos;

	physRot(float angle = 0);

	inline void set(float angle);
	inline void setIdentity();
	inline void rotateBy(float angle);
	inline float angle() const;
	inline physVec2 xAxis() const;
	inline physVec2 yAxis() const;
	inline physRot operator-() const;
};

#pragma region physRot definitions
inline physRot::physRot(float angle)
{
	set(angle);
}

inline void physRot::set(float angle)
{
	sin = sinf(angle);
	cos = cosf(angle);
}
inline void physRot::setIdentity()
{
	sin = 0.0f;
	cos = 1.0f;
}
inline void physRot::rotateBy(float angle)
{
	set(this->angle() + angle);
}
inline float physRot::angle() const
{
	return atan2f(sin, cos);
}
inline physVec2 physRot::xAxis() const
{
	return { cos, sin };
}
inline physVec2 physRot::yAxis() const
{
	return { -sin, cos };
}
inline physRot physRot::operator-() const
{
	return physRot(-this->angle());
}
#pragma endregion

#pragma region physVec2 definitions

inline physVec2::physVec2(float x, float y) : x(x), y(y)
{
}

inline float physVec2::lengthSquared() const
{
	return (x*x) + (y*y);
}
inline float physVec2::length() const
{
	return sqrt(lengthSquared());
}
inline float physVec2::dot(const physVec2& rhs) const
{
	return (this->x * rhs.x) + (this->y * rhs.y);
}
inline float physVec2::cross(const physVec2& rhs) const
{
	return (this->x * rhs.y) - (this->y * rhs.x);
}
inline physVec2 physVec2::cross(float scalar) const
{
	return{ scalar * this->y, -scalar * this->x };
}
inline void physVec2::normalize()
{
	float l = length();
	if (l != 0)
	{
		this->x /= l;
		this->y /= l;
	}
	else
	{
		this->x = 0;
		this->y = 0;
	}
}
inline void physVec2::rotate(const physRot & rot)
{
	physVec2 temp = *this;
	temp.x = this->x * rot.cos - this->y * rot.sin;
	temp.y = this->x * rot.sin + this->y * rot.cos;
	*this = temp;
}
inline physVec2 physVec2::operator+(const physVec2& rhs) const
{
	return{ this->x + rhs.x, this->y + rhs.y };
}
inline physVec2 physVec2::operator-(const physVec2& rhs) const
{
	return{ this->x - rhs.x, this->y - rhs.y };
}
inline physVec2 physVec2::operator-() const
{
	return physVec2(-x, -y);
}
inline physVec2& physVec2::operator+=(const physVec2& rhs)
{
	this->x += rhs.x;
	this->y += rhs.y;
	return *this;
}
inline physVec2& physVec2::operator-=(const physVec2& rhs)
{
	this->x -= rhs.x;
	this->y -= rhs.y;
	return *this;
}
inline bool physVec2::operator<(const physVec2& rhs) const
{
	return this->length() < rhs.length();
}
inline bool physVec2::operator>(const physVec2& rhs) const
{
	return rhs < *this;
}
inline bool physVec2::operator<=(const physVec2& rhs) const
{
	return !(*this > rhs);
}
inline bool physVec2::operator>=(const physVec2& rhs) const
{
	return !(*this < rhs);
}
inline physVec2 physVec2::operator*(const float scalar) const
{
	return{ this->x * scalar, this->y * scalar };
}
inline physVec2 physVec2::operator/(const float scalar) const
{
	return{ this->x / scalar, this->y / scalar };
}

#pragma endregion

#pragma region utility
inline physVec2 minVec2(const physVec2& lhs, const physVec2& rhs)
{
	return { std::min(lhs.x, rhs.x), std::min(lhs.y, rhs.y) };
}

inline physVec2 maxVec2(const physVec2& lhs, const physVec2& rhs)
{
	return{ std::max(lhs.x, rhs.x), std::max(lhs.y, rhs.y) };
}

inline physVec2 clampVec2(const physVec2& in, const physVec2& low, const physVec2& high)
{
	return maxVec2(low, minVec2(in, high));
}

inline physVec2 tripleProduct(const physVec2& v1, const physVec2& v2, const physVec2& v3)
{
	physVec2 result;
	result = v3.cross(-v1.cross(v2));
	return result;
}
#pragma endregion