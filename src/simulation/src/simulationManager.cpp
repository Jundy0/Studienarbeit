#include "simulationManager.h"

#include <iostream>

SimulationManager::SimulationManager()
{
    this->window = new sf::RenderWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulation", sf::Style::Titlebar | sf::Style::Close);
    this->vehicle = new Vehicle(5.f, 5.f);

    Obstacle obstacle = Obstacle(100.f, 100.f, 50.f, 50.f);
    Obstacle obstacle2 = Obstacle(190.f, 170.f, 100.f, 50.f);

    this->obstacles.push_back(obstacle);
    this->obstacles.push_back(obstacle2);
}

SimulationManager::~SimulationManager()
{
    delete this->window;
    delete this->vehicle;
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

    this->vehicle->update();

    // Check for collisions
    for (auto &obstacle : this->obstacles)
    {
        if (obstacle.getPosition().intersects(this->vehicle->getPosition()))
        {
            std::cout << "Error: Vehicle crashed into an obstacle!" << std::endl;
        }
    }
}

void SimulationManager::render()
{
    window->clear(sf::Color::Cyan);

    for (auto &obstacle : this->obstacles)
    {
        window->draw(obstacle.getShape());
    }

    window->draw(this->vehicle->getShape());

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
                this->vehicle->moveLeft();
                break;
            case sf::Keyboard::Key::D:
                this->vehicle->moveRight();
                break;
            case sf::Keyboard::Key::W:
                this->vehicle->moveUp();
                break;
            case sf::Keyboard::Key::S:
                this->vehicle->moveDown();
                break;
            }
            break;
        }
    }
}
