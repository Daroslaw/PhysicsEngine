#include "Game.h"
#include "SFML/System.hpp"

float randf(int rb, int lb = 0)
{
    return static_cast<float>((rand() % (rb - lb)) + lb);
}


void DrawAABB(physAABB aabb, sf::Color color, sf::RenderWindow & rw)
{
    sf::RectangleShape rect;
    rect.setFillColor(sf::Color::Transparent);
    rect.setOutlineColor(color);
    rect.setOutlineThickness(1);
    rect.setPosition(sf::Vector2f(aabb.GetCenter().x, aabb.GetCenter().y));
    rect.setSize(sf::Vector2f(aabb.GetExtents().x * 2, aabb.GetExtents().y * 2));
    rect.setOrigin(sf::Vector2f(aabb.GetExtents().x, aabb.GetExtents().y));

    rw.draw(rect);
}

void DrawCircle(physBody *b, sf::RenderWindow & rw)
{
    sf::CircleShape circle;
    physCircleShape *c = dynamic_cast<physCircleShape*>(b->GetShape());
    circle.setOrigin(sf::Vector2f(c->GetRadius() - 1, c->GetRadius() - 1));
    circle.setPosition(sf::Vector2f(b->GetPos().x, b->GetPos().y));
    circle.setRadius(c->GetRadius() - 2);
    circle.setFillColor(sf::Color::White);
    circle.setOutlineThickness(2);
    circle.setOutlineColor(sf::Color(128, 128, 128, 255));
    circle.setRotation(b->GetRot().angle());

    sf::RectangleShape line;
    line.setPosition(sf::Vector2f(b->GetPos().x, b->GetPos().y));
    line.setSize(sf::Vector2f(1, c->GetRadius() - 2));
    line.setOrigin(sf::Vector2f(1, c->GetRadius() - 2));
    line.setOutlineThickness(1);
    line.setOutlineColor(sf::Color(128, 128, 128, 255));
    line.setFillColor(sf::Color(128, 128, 128, 255));
    line.setRotation(180.f * b->GetRot().angle() / static_cast<float>(M_PI));

    rw.draw(circle);
    rw.draw(line);
}

void DrawPoly(physBody *b, sf::RenderWindow & rw)
{
    sf::ConvexShape poly;
    physPolyShape *p = dynamic_cast<physPolyShape*>(b->GetShape());
    physPolyHandle &ph = p->GetPolygon();

    poly.setPointCount(ph.GetCount());
    for (unsigned int i = 0; i < ph.GetCount(); ++i)
        poly.setPoint(i, sf::Vector2f(ph.At(i).position.x, ph.At(i).position.y));
    poly.setPosition(sf::Vector2f(b->GetPos().x, b->GetPos().y));
    poly.setFillColor(sf::Color::White);
    poly.setOutlineThickness(-2);
    poly.setOutlineColor(sf::Color(128, 128, 128, 255));
    poly.setRotation(180.f * b->GetRot().angle() / static_cast<float>(M_PI));

    rw.draw(poly);
}



void Game::Init()
{
    drawAABB = false;
    drawContactPoints = false;
    InitScene1();
}

void Game::InitScene1()
{
    world.SetGravity({ 0.f, 10.f });

    auto floor1 = world.RegisterRectangle(200, 200, 200, 30);
    floor1->SetStatic();
    floor1->SetRotation(static_cast<float>(M_PI_4) / 2);
    bodies.push_back(floor1);
    auto floor2 = world.RegisterRectangle(400, 300, 200, 30);
    floor2->SetStatic();
    floor2->SetStaticFriction(0.9f);
    floor2->SetDynamicFriction(0.6f);
    floor2->SetRotation(-static_cast<float>(M_PI_4) / 2);
    bodies.push_back(floor2);
    auto floor3 = world.RegisterRectangle(200, 400, 200, 30);
    floor3->SetStatic();
    floor3->SetRotation(static_cast<float>(M_PI_4) / 2);
    bodies.push_back(floor3);
    auto circle = world.RegisterCircle(130, 10, 20);
    circle->SetRestitution(0.5f);
    bodies.push_back(circle);
    auto box = world.RegisterBox(130, 70, 30);
    box->SetRestitution(0.1f);
    box->SetStaticFriction(0.2f);
    box->SetDynamicFriction(0.1f);
    bodies.push_back(box);
}

void Game::InitScene2()
{
    world.SetGravity({ 0, 10 });

    auto floor = world.RegisterRectangle(400, 600, 800, 100);
    floor->SetStatic();
    bodies.push_back(floor);

    auto ball = world.RegisterCircle(100, 500, 20);
    bodies.push_back(ball);
    ball->ApplyImpulse({ 50000, -70000 });
}

void Game::InitScene3()
{
    world.SetGravity({ 0, 10 });

    auto floor = world.RegisterRectangle(400, 300, 300, 100);
    floor->SetStatic();
    bodies.push_back(floor);

    //auto box = world.RegisterRectangle(400, 200, 30, 30);
    //box->SetRotation(M_PI_4 / 2);
    //bodies.push_back(box);

    bodies.push_back(world.RegisterRectangle(600, 100, 250, 80));
    bodies.push_back(world.RegisterRectangle(450, 0, 300, 100));
}

void Game::InitScene4()
{
    world.SetGravity({ 0, 0 });

    auto brick = world.RegisterRectangle(200, 100, 100, 30);
    bodies.push_back(brick);

    auto ball = world.RegisterCircle(320, 220, 20);
    bodies.push_back(ball);

    ball->SetDensity(3.f);
    brick->ApplyImpulse({ 40000, 40000 }, { -150, -50 });
    ball->ApplyImpulse({ -40000, -40000 }, { -10, -10 });

}

void Game::InitScene5()
{
    world.SetGravity({ 0, 10 });

    physVec2 v[5] = { { -50, -50 },{ 50, -50 },{ 100, 0 },{ 50, 50 },{ -50, 50 } };
    auto bullet = world.RegisterPoly(-100, 300, v, 5);
    bodies.push_back(bullet);

    for (int i = 0; i < 498; ++i)
    {
        //bodies.push_back(world.RegisterCircle(randf(800), randf(400, 200), randf(40, 10)));
        bodies.push_back(world.RegisterBox(randf(800), randf(400, 200), randf(40, 10)));
    }

    bullet->ApplyImpulse({ 5000000, 0 });
    bullet->SetDensity(100.f);

    //auto floor = world.RegisterRectangle(400, 600, 800, 100);
    //floor->SetStatic();
    //bodies.push_back(floor);
    //
    //auto wall = world.RegisterRectangle(800, 300, 100, 600);
    //wall->SetStatic();
    //bodies.push_back(wall);
}

void Game::InitScene6()
{
    world.SetGravity({ 0, 10 });

    auto floor = world.RegisterRectangle(400, 300, 100, 100);
    floor->SetStatic();
    bodies.push_back(floor);

    //auto box = world.RegisterRectangle(400, 200, 30, 30);
    //box->SetRotation(M_PI_4 / 2);
    //bodies.push_back(box);

    bodies.push_back(world.RegisterBox(400, 200, 30));
    //bodies.push_back(world.RegisterBox(400, 160, 30));
    //bodies.push_back(world.RegisterBox(400, 120, 30));
    //bodies.back()->ApplyImpulse({ 0, 10000 });
}

void Game::InitScene7()
{
    world.SetGravity({ 0, 10 });

    auto floor = world.RegisterRectangle(400, 600, 1000, 100);
    floor->SetStatic();
    bodies.push_back(floor);

    for (int i = 0; i < 5; ++i)
    {
        for (int j = 5 - i; j > 0; --j)
        {
            bodies.push_back(world.RegisterBox(100.f + (i * 16.f) + (j * 32.f), 530.f - (i * 32.f), 30.f));
            bodies.back()->SetStaticFriction(0.7f);
            bodies.back()->SetDynamicFriction(0.4f);
        }
    }

    //auto ball = world.RegisterCircle(200, 0, 50);
    //ball->SetDensity(10.f);
    //bodies.push_back(ball);
}

void Game::Update(const float dt)
{
    world.Simulate();
}

void Game::Render(sf::RenderWindow & rw)
{
    rw.clear();
    for (auto b : bodies)
    {
        physShape *s = b->GetShape();
        switch (s->GetType())
        {
        case physShape::sCircle:
        {
            DrawCircle(b, rw);
            break;
        }
        case physShape::sPoly:
        {
            DrawPoly(b, rw);
            break;
        }
        default:
            break;
        }
        if (drawAABB)
            DrawAABB(b->GetAABB(), sf::Color::Red, rw);

    }
    for (int i = 0; i < world.GetCollisions()->count; ++i)
    {
        auto c = world.GetCollisions()->collisions[i];
        if (!drawAABB && !drawContactPoints)
            continue;
        physAABB aabb = c.A->GetAABB().Combine(c.B->GetAABB());
        if (c.contactCnt)
        {
            if (drawAABB)
                DrawAABB(aabb, sf::Color::Green, rw);
            if (drawContactPoints)
            {
                for (int i = 0; i < c.contactCnt; ++i)
                {
                    physVec2 cpPos = c.contacts[i];
                    sf::CircleShape cp;
                    cp.setRadius(3);
                    cp.setFillColor(sf::Color::Red);
                    cp.setPosition(cpPos.x, cpPos.y);
                    cp.setOrigin(1.5, 1.5);
                    rw.draw(cp);
                }
            }
        }
        else if (drawAABB)
            DrawAABB(aabb, sf::Color::Magenta, rw);
    }
    rw.display();
}

void Game::Input(sf::Keyboard::Key key)
{
    switch (key)
    {
    case sf::Keyboard::Num1:
        Clear();
        InitScene1();
        break;
    case sf::Keyboard::Num2:
        Clear();
        InitScene2();
        break;
    case sf::Keyboard::Num3:
        Clear();
        InitScene3();
        break;
    case sf::Keyboard::Num4:
        Clear();
        InitScene4();
        break;
    case sf::Keyboard::Num5:
        Clear();
        InitScene5();
        break;
    case sf::Keyboard::Num6:
        Clear();
        InitScene6();
        break;
    case sf::Keyboard::Num7:
        Clear();
        InitScene7();
        break;
    case sf::Keyboard::B:
        drawAABB = !drawAABB;
        break;
    case sf::Keyboard::P:
        drawContactPoints = !drawContactPoints;
        break;
    case sf::Keyboard::Up:
        bodies.back()->ApplyForce({ 0, -50000 });
        break;
    case sf::Keyboard::Down:
        bodies.back()->ApplyForce({ 0, 50000 });
        break;
    case sf::Keyboard::Left:
        bodies.back()->ApplyForce({ -50000, 0 });
        break;
    case sf::Keyboard::Right:
        bodies.back()->ApplyForce({ 50000, 0 });
        break;
    default:
        break;
    }
}

void Game::Clear()
{
    world.DestroyAll();
    bodies.clear();
}
