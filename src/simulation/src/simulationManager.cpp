#include "simulationManager.h"

#include <iostream>

SimulationManager::SimulationManager()
{
    this->window = new sf::RenderWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulation", sf::Style::Titlebar | sf::Style::Close);
}

SimulationManager::~SimulationManager()
{
    delete this->window;
}

void SimulationManager::run()
{
    this->window->setFramerateLimit(60);

    while (this->window->isOpen())
    {
        this->update();
        this->render();
    }
}

void SimulationManager::update()
{
    this->pollEvent();

    // Update and Animations
}

void SimulationManager::render()
{
    window->clear(sf::Color(255, 0, 0, 255));

    // Draw Stuff

    window->display();
}

void SimulationManager::pollEvent()
{
    while (this->window->pollEvent(this->ev))
    {
        switch (this->ev.type)
        {
        case sf::Event::Closed:
            this->window->close();
            break;
        case sf::Event::KeyPressed:
            std::cout << this->ev.key.code << std::endl;
            break;
        }
    }
}
