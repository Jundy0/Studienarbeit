#include "simulationManager.h"
#include "vehicle.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <cmath>

#define WINDOW_WIDTH 1000
#define WINDOW_HEIGHT 1000

#define GRID_WIDTH 1000
#define GRID_HEIGHT 1000

#define COUNT 720
#define RADIUS 5

#define PROB_OCC 0.4
#define PROB_FREE -0.2

SimulationManager::SimulationManager()
{
    this->window = new sf::RenderWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulation", sf::Style::Titlebar | sf::Style::Close);
    this->window->setFramerateLimit(60);
    this->windowImage.create(WINDOW_WIDTH, WINDOW_HEIGHT, sf::Color::Transparent);
    this->windowTexture.loadFromImage(this->windowImage);
    this->windowSprite.setTexture(this->windowTexture, true);
    this->font.loadFromFile("../res/Arial.ttf");

    this->vehicle = new Vehicle((float)WINDOW_WIDTH / 2, (float)WINDOW_HEIGHT / 2);
}

SimulationManager::~SimulationManager()
{
    delete this->window;
    delete this->vehicle;
}

void SimulationManager::render(Eigen::MatrixXd gridMap, Eigen::Vector2i robPos, double robRotation)
{
    window->clear(sf::Color::Cyan);

    const sf::FloatRect vehicleRect = this->vehicle->getPosition();

    int gridX;
    int gridY;

    sf::Color color;

    for (int x = 0; x < WINDOW_WIDTH; x++)
    {
        for (int y = 0; y < WINDOW_HEIGHT; y++)
        {
            gridX = round(x / WINDOW_WIDTH / GRID_WIDTH);
            gridY = round(y / WINDOW_HEIGHT / GRID_HEIGHT);

            if (gridMap(gridY, gridX) >= PROB_OCC)
            {
                color = sf::Color::Black;
            }
            else if (gridMap(gridY, gridX) <= PROB_FREE)
            {
                color = sf::Color::Black;
            }
            else
            {
                color = sf::Color::Cyan;
            }
            windowImage.setPixel(x, y, color);
        }
    }

    this->vehicle->setPosition(sf::Vector2f{(float)robPos[0], (float)robPos[1]});
    this->vehicle->setRotation(robRotation);
    this->vehicle->update();

    windowTexture.loadFromImage(windowImage);

    window->draw(this->windowSprite);

    window->display();
}