#include "PhysicsPoly.h"

physVertex::physVertex()
{
	position = { 0, 0 };
	normal = { 0, 0 };
}

physPolyHandle::physPolyHandle()
{
	this->m_vertArray = physPolyHandler::dummyHandle.m_vertArray;
	this->m_vertCount = physPolyHandler::dummyHandle.m_vertCount;
}

physPolyHandle::physPolyHandle(physVertex * firstVertex, unsigned int count) : m_vertArray(firstVertex), m_vertCount(count)
{
}

physPolyHandler::physPolyHandler()
{
	m_tailIdx = 0;
}

physVertex physPolyHandler::m_dummyVertex = physVertex();
const physPolyHandle physPolyHandler::dummyHandle = physPolyHandle(&physPolyHandler::m_dummyVertex, 1);

physPolyHandle physPolyHandler::CreatePoly(physVec2 *vertices, unsigned int count)
{
	if (m_tailIdx + count >= MAX_VERTICES)
		return dummyHandle;
	
	physPolyHandle newHandle(&m_allVertices[m_tailIdx], count);
	for (unsigned int i = 0; i < count; ++i)
		m_allVertices[i + m_tailIdx].position = vertices[i];
	for (unsigned int i = 0; i < count; ++i)
	{
		physVec2 face = m_allVertices[((i + 1) % count) + m_tailIdx].position - m_allVertices[i + m_tailIdx].position;
		m_allVertices[i + m_tailIdx].normal = physVec2(face.y, -face.x);
		m_allVertices[i + m_tailIdx].normal.normalize();
	}
	m_tailIdx += count;
	return newHandle;
}

void physPolyHandler::DestroyAll()
{
	for (uint32_t i = 0; i < m_tailIdx; ++i)
		m_allVertices[i] = m_dummyVertex;
	m_tailIdx = 0;
}

void physPolyHandler::DestroyPoly(const physPolyHandler & polygon)
{
	//	TODO
}

