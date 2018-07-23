#pragma once

#include "PhysicsBody.h"
#include "PhysicsCollision.h"
#include <vector>

template<uint32_t N>
constexpr uint32_t POW_OF_4()
{
    return 1 << N << N;
}

constexpr uint8_t QUAD_TREE_LEVELS = 3;
constexpr uint32_t QUAD_TREE_NODES = (POW_OF_4<QUAD_TREE_LEVELS>() - 1) / 3;

struct Node
{
    physVec2 center;
    float halfWidth;
    Node *children[4] = {nullptr};
    std::vector<physBody*> bodies;
};

//  TREE WINDING: NW -> SW -> SE -> NE

class BroadPhaseQuadTree
{
public:
    BroadPhaseQuadTree(physVec2 treeCenter, float treeHalfWidth) :
        m_treeCenter(treeCenter),
        m_treeHalfWidth(treeHalfWidth)
    {
        BuildTree(m_treeCenter, m_treeHalfWidth);
    }

    void SetBodies(physBodyBufferSpan & bodies)
    {
        for(uint32_t i = 0; i < bodies.size; ++i)
            InsertObject(&bodies[i], m_treeArray);
    }

    void Solve(physCollisionBuffer & collisions)
    {
        //SolveCollisions(m_treeArray, collisions);
        SolveCollisions(collisions);
        Clear();
    }

private:
    void BuildTree(physVec2 nodeCenter, float nodeHalfWidth, uint32_t nodeIdx = 0)
    {
        Node & currentNode = m_treeArray[nodeIdx];

        currentNode.center = nodeCenter;
        currentNode.halfWidth = nodeHalfWidth;
        
        uint32_t childIdx = nodeIdx * 4;
        if (childIdx + 4 >= QUAD_TREE_NODES)
            return;

        float halfWidthHalved = nodeHalfWidth * 0.5f;

        currentNode.children[0] = &m_treeArray[++childIdx];
        BuildTree(nodeCenter + physVec2(-halfWidthHalved, -halfWidthHalved), halfWidthHalved, childIdx);
        currentNode.children[1] = &m_treeArray[++childIdx];
        BuildTree(nodeCenter + physVec2(-halfWidthHalved, halfWidthHalved), halfWidthHalved, childIdx);
        currentNode.children[2] = &m_treeArray[++childIdx];
        BuildTree(nodeCenter + physVec2(halfWidthHalved, halfWidthHalved), halfWidthHalved, childIdx);
        currentNode.children[3] = &m_treeArray[++childIdx];   
        BuildTree(nodeCenter + physVec2(halfWidthHalved, -halfWidthHalved), halfWidthHalved, childIdx);
    }

    void InsertObject(physBody * body, Node * node) const
    {
        uint8_t index = 0;
        bool stayOnThisLevel = false;

        physVec2 delta = body->GetPos() - node->center;

        float bodyFullExtents = (body->GetAABB().GetExtents() * 2).length();
        if (delta.length() < /*node->halfWidth + */bodyFullExtents)
        {
            stayOnThisLevel = true;
        }
        else
        {
            if (delta.y) index += 1;
            if (delta.x > 0) index += 2;
        }
        
        if (!stayOnThisLevel && node->children[index])
            InsertObject(body, node->children[index]);
        else
            node->bodies.push_back(body);
    }

    /*void SolveCollisions(Node * node, physCollisionBuffer & collisions) const
    {
        constexpr uint16_t MAX_DEPTH = 40;
        static Node *ancestorStack[MAX_DEPTH];
        static uint16_t depth = 0;

        ancestorStack[depth++] = node;
        for(uint32_t i = 0; i < depth; ++i)
        {
            for(auto bodyA : ancestorStack[i]->bodies)
            {
                for(auto bodyB : node->bodies)
                {
                    if (bodyA == bodyB)
                        break;
                    collisions.AppendCollision(bodyA, bodyB);
                }
            }
        }

        for (uint32_t i = 0; i < 4; ++i)
            if (node->children[i])
                SolveCollisions(node->children[i], collisions);

        --depth;
    }*/

    void SolveCollisions(physCollisionBuffer & collisions, uint32_t nodeIdx = 0)
    {
        auto & currentNode = m_treeArray[nodeIdx];
        uint32_t otherNodeIdx = nodeIdx;
        while(true)
        {
            auto & otherNode = m_treeArray[otherNodeIdx];
            for(auto & bodyA : currentNode.bodies)
            {
                for(auto & bodyB : otherNode.bodies)
                {
                    if (bodyA == bodyB)
                        break;
                    auto &aabb1 = bodyA->GetAABB();
                    auto &aabb2 = bodyB->GetAABB();
                    if (aabb1.Intersects(aabb2))
                        collisions.AppendCollision(bodyA, bodyB);
                }
            }
            int32_t breakFlag = int32_t(otherNodeIdx) - 1;
            if (breakFlag < 0) break;
            otherNodeIdx = (otherNodeIdx - 1) / 4;
        }

        for (uint32_t i = (nodeIdx * 4) + 1; i <= (nodeIdx + 1) * 4; ++i)
        {
            if (i >= QUAD_TREE_NODES)
                break;
            SolveCollisions(collisions, i);
        }
    }

    void Clear()
    {
        for (uint32_t i = 0; i < QUAD_TREE_NODES; ++i)
            m_treeArray[i].bodies.clear();
    }

    Node m_treeArray[QUAD_TREE_NODES];
    physVec2 m_treeCenter;
    float m_treeHalfWidth;
};