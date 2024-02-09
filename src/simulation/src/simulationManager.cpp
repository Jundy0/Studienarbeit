#include "simulationManager.h"

#include <iostream>

SimulationManager::SimulationManager()
{
    this->window = new sf::RenderWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulation", sf::Style::Titlebar | sf::Style::Close);
    this->vehicle = Vehicle(5.f, 5.f);
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

    this->vehicle.update();

    // Update and Animations
}

void SimulationManager::render()
{
    window->clear(sf::Color::Cyan);

    // Draw Stuff

    window->draw(this->vehicle.getShape());

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
            switch (this->ev.key.code)
            {
            case sf::Keyboard::Key::A:
                this->vehicle.moveLeft();
                break;
            case sf::Keyboard::Key::D:
                this->vehicle.moveRight();
                break;
            case sf::Keyboard::Key::W:
                this->vehicle.moveUp();
                break;
            case sf::Keyboard::Key::S:
                this->vehicle.moveDown();
                break;
            }
            break;
        }
    }
}
