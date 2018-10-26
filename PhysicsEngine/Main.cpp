#include <iostream>
#include "SFML/System.hpp"
#include "SFML/Graphics.hpp"
#include "Statistics.h"
#include <iomanip>
#include "Game.h"
#include "Benchmark.h"

constexpr float FPS = 180.f;
constexpr float speedFactor = 1;

constexpr unsigned BENCHMARK_BUFFER_SIZE = 100;

void PrintBenchmark()
{
    using namespace std;

    static MovingAverage maCnt(BENCHMARK_BUFFER_SIZE);
    static MovingAverage maMs(BENCHMARK_BUFFER_SIZE);
    static MovingAverage testCnt(BENCHMARK_BUFFER_SIZE);
    static MovingAverage allCol(BENCHMARK_BUFFER_SIZE);
    static MovingAverage filtrCol(BENCHMARK_BUFFER_SIZE);
    static MovingAverage sizes(BENCHMARK_BUFFER_SIZE);

    auto result =  Benchmark::Get().PopResult("Collision");
    auto resultMs = result.milli();

    auto getVal = [](const std::string& valName) { return Benchmark::Get().PopValue(valName); };

    auto testCount = getVal("TestCount");
    auto allCols = getVal("AllCollisions");
    auto uqCols = getVal("UniqueCollisions");
    auto size = getVal("Size");

    cout << "#####"                                                 << endl;
    cout << "Cycles: \t" <<                 result.count            << endl;
    cout << "Cycles avg: \t" <<             maCnt.Get(result.count) << endl;
    cout << setprecision(3);
    cout << "Ms: \t \t" <<                  resultMs                << endl;
    cout << "Avg Ms: \t" <<                 maMs.Get(resultMs)      << endl;
    cout << setprecision(2);
    cout << "Number of tests: \t" <<        testCount               << endl;
    cout << "Number of tests avg: \t" <<    testCnt.Get(testCount)  << endl;
    cout << "All collisions: \t" <<         allCols                 << endl;
    cout << "All collisions avg: \t" <<     allCol.Get(allCols)     << endl;
    cout << "Unique collisions: \t" <<      uqCols                  << endl;
    cout << "Unique collisions avg: \t" <<  filtrCol.Get(uqCols)    << endl;
    cout << "Size: \t \t \t" <<             size                    << endl;
    cout << "Size avg: \t \t" <<            sizes.Get(size) << endl << endl;
}

void main(void)
{
	srand(static_cast<int>(time(nullptr)));

	sf::RenderWindow simulationWindow(sf::VideoMode(800, 800), "Physics Engine");
	simulationWindow.setFramerateLimit(FPS);

	sf::Time frameTime = sf::seconds(speedFactor / FPS);
	sf::Clock fpsCounter;

	Game *game = new Game();
	game->Init();

    std::cout << std::fixed << std::setprecision(2);

    int iterations = 0;

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
				game->Input(ev.key.code);
				break;
			default:
				break;
			}
		}
        Benchmark::Get().RunTimer("Total");
        Benchmark::Get().RunTimer("Update");
		game->Update(frameTime.asSeconds());
        Benchmark::Get().StopTimer("Update");
        Benchmark::Get().RunTimer("Render");
        game->Render(simulationWindow);
        Benchmark::Get().StopTimer("Render");
        Benchmark::Get().StopTimer("Total");

        PrintBenchmark();
        //if (iterations++ >= BENCHMARK_BUFFER_SIZE + 5)
        //    simulationWindow.close();
	}
    delete game;
}