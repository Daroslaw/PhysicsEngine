#pragma once

constexpr float DT = 1.f / 20.f;
constexpr unsigned int MAX_BODIES = 500;

constexpr unsigned int MAX_CIRCLES = MAX_BODIES;
constexpr unsigned int MAX_POLYGONS = MAX_BODIES;
constexpr unsigned int MAX_COLLISIONS = 15000000;

#define BP_NAIVE 0
#define BP_UNIFORM 8
#define BP_UNIFORM_FIXED 9
#define BP_UNIFORM_OPENHASH 10
#define BP_UNIFORM_CLOSEDHASH 11
#define BP_HIERARCHICAL_GRID 4
#define BP_QUADTREE 5

#define BP BP_QUADTREE