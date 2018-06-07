#pragma once

#include <vector>
#include "PhysicsWorld.h"
#include "SFML/Graphics.hpp"

struct Game
{
    physWorld world;
    bool drawAABB = false;
    bool drawContactPoints = false;

    std::vector<physBody*> bodies;

    void Init();

    void InitScene1();
    void InitScene2();
    void InitScene3();
    void InitScene4();
    void InitScene5();

    void InitScene6();
    void InitScene7();

    void Update(const float dt);
    void Render(sf::RenderWindow & rw);
    void Input(sf::Keyboard::Key key);
    void Clear();

    Game() {}
};
