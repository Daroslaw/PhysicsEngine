#include "PhysicsShape.h"

float physShape::GetArea()
{
	if (m_area == 0) ComputeArea();
	return m_area;
}

physCircleShape::physCircleShape()
{
	m_radius = 0;
	m_area = 0;
}

physCircleShape::physCircleShape(float radius) : m_radius(radius)
{
	m_radius = radius;
}

void physCircleShape::Initialize()
{
	ComputeArea();
}

physShape::Type physCircleShape::GetType()
{
	return sCircle;
}

float physCircleShape::GetRadius() const
{
	return m_radius;
}

void physCircleShape::ComputeArea()
{
	m_area = m_radius * m_radius * static_cast<float>(M_PI);
}

physPolyShape::physPolyShape()
{
	m_area = 0;
}

physPolyShape::physPolyShape(physPolyHandle polygon) : m_polygon(polygon)
{
}

void physPolyShape::Initialize()
{
	ComputeArea();
	AdjustByCentroid();
}

physShape::Type physPolyShape::GetType()
{
	return sPoly;
}

physPolyHandle & physPolyShape::GetPolygon()
{
	return m_polygon;
}

void physPolyShape::ComputeArea()
{
	m_area = 0;

	for (unsigned int i = 0; i < m_polygon.GetCount(); ++i)
	{
		physVec2 v1 = m_polygon.At(i).position;
		physVec2 v2 = m_polygon.At(i + 1).position;

		m_area += v1.cross(v2);
	}
	m_area /= 2;
	m_area = (m_area < 0) ? abs(m_area) : m_area;
}

void physPolyShape::AdjustByCentroid() const
{
	// http://www.seas.upenn.edu/~sys502/extra_materials/Polygon%20Area%20and%20Centroid.pdf
	
	if (m_area == 0) return;

	float cx = 0;
	float cy = 0;

	for (unsigned int i = 0; i < m_polygon.GetCount(); ++i)
	{
		physVec2 v1 = m_polygon.At(i).position;
		physVec2 v2 = m_polygon.At(i + 1).position;
		float x = v1.cross(v2);

		cx += (v1.x + v2.x) * x;
		cy += (v1.y + v2.y) * x;
	}

	cx /= 6 * m_area;
	cy /= 6 * m_area;

	physVec2 centroid = { cx, cy };

	for (unsigned int i = 0; i < m_polygon.GetCount(); ++i)
		m_polygon.At(i).position += centroid;
}


