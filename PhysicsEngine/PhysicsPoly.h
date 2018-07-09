#pragma once

#include "PhysicsMath.h"

constexpr unsigned int MAX_VERTICES = 20100;

struct physVertex
{
	physVertex();
	physVec2 position;
	physVec2 normal;
};

class physPolyHandle
{
public:
	physPolyHandle();
	physPolyHandle(physVertex* firstVertex, unsigned int count);

	physVertex& At(unsigned int i) const { return m_vertArray[i % m_vertCount]; }
	unsigned int GetCount() const { return m_vertCount; }

private:
	physVertex* m_vertArray;
	int m_vertCount;
};

class physPolyHandler
{
public:
	physPolyHandler();

	static const physPolyHandle dummyHandle;

	physPolyHandle CreatePoly(physVec2 *vertices, unsigned int count);

	void DestroyAll();
	void DestroyPoly(const physPolyHandler& polygon);

private:
	physVertex m_allVertices[MAX_VERTICES];
	static physVertex m_dummyVertex;
	unsigned int m_tailIdx;
};