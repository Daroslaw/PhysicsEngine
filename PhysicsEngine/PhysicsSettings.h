#pragma once

constexpr float DT = 1.f / 20.f;
constexpr unsigned int MAX_BODIES = 500;

constexpr unsigned int MAX_CIRCLES = MAX_BODIES;
constexpr unsigned int MAX_POLYGONS = MAX_BODIES;
constexpr unsigned int MAX_COLLISIONS = 20 * MAX_BODIES;