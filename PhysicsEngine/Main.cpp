#include <iostream>
#include "SFML/System.hpp"
#include "SFML/Graphics.hpp"
#include "Benchmarking.h"
#include <iomanip>
#include "Game.h"
#include "Benchmark.h"

constexpr float FPS = 180.f;
constexpr float speedFactor = 1;

void main(void)
{
	srand(static_cast<int>(time(nullptr)));

	sf::RenderWindow simulationWindow(sf::VideoMode(800, 600), "Physics Engine");
	simulationWindow.setFramerateLimit(FPS);

    //sf::RenderWindow benchmarkWindow(sf::VideoMode(320, 240), "Benchmark");

    bm::MovingAverage ma(32);
	sf::Time frameTime = sf::seconds(speedFactor / FPS);
	sf::Clock fpsCounter;

	Game game;
	game.Init();

    std::cout << std::setprecision(9);

	while (simulationWindow.isOpen())
	{
        double fps = 1.f / fpsCounter.restart().asSeconds();

		sf::Event ev;
		while (simulationWindow.pollEvent(ev))
		{
			switch(ev.type)
			{
			case sf::Event::Closed:
				simulationWindow.close();
				break;
			case sf::Event::KeyPressed:
				game.Input(ev.key.code);
				break;
			default:
				break;
			}
		}
        Benchmark::Get().RunTimer("Total");
        Benchmark::Get().RunTimer("Update");
		game.Update(frameTime.asSeconds());
        Benchmark::Get().StopTimer("Update");
        Benchmark::Get().RunTimer("Render");
        //game.Render(simulationWindow);
        Benchmark::Get().StopTimer("Render");
        Benchmark::Get().StopTimer("Total");

        std::cout << "#####" << std::endl;
        std::cout << "Total: " << Benchmark::Get().PopResult("Total").count << std::endl;
        std::cout << "Update: " << Benchmark::Get().PopResult("Update").count << std::endl;
        std::cout << "Render: " << Benchmark::Get().PopResult("Render").count << std::endl;
        std::cout << "Collision: " << Benchmark::Get().PopResult("Collision").count << std::endl << std::endl;
	}
    //benchmarkWindow.close();
}