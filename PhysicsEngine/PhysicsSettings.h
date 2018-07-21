#pragma once

constexpr float DT = 1.f / 20.f;
constexpr unsigned int MAX_BODIES = 2000;

constexpr unsigned int MAX_CIRCLES = MAX_BODIES;
constexpr unsigned int MAX_POLYGONS = MAX_BODIES;
constexpr unsigned int MAX_COLLISIONS = MAX_BODIES * 1000;