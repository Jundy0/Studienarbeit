#include "simulationManager.h"
#include "lidarSensorSim.h"

#include <iostream>

#define COUNT 360
#define RADIUS 5

SimulationManager::SimulationManager()
{
    this->window = new sf::RenderWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulation", sf::Style::Titlebar | sf::Style::Close);
    this->vehicle = new Vehicle(5.f, 5.f);

    Obstacle obstacle = Obstacle(100.f, 100.f, 50.f, 50.f);
    Obstacle obstacle2 = Obstacle(190.f, 170.f, 100.f, 50.f);

    this->obstacles.push_back(obstacle);
    this->obstacles.push_back(obstacle2);

    this->lidarSensor = new LidarSensorSim(this->vehicle, this->obstacles);

    this->data = (lidar_point_t *)malloc(sizeof(lidar_point_t) * COUNT);
}

SimulationManager::~SimulationManager()
{
    delete this->window;
    delete this->vehicle;

    free(this->data);
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

    this->lidarSensor->getScanData(this->data, COUNT);

    for (size_t i = 0; i < COUNT; i++)
    {
        // std::cout << data[i].angle << "; " << data[i].radius << std::endl;
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

    sf::CircleShape circle = sf::CircleShape(RADIUS);
    circle.setFillColor(sf::Color::Red);

    for (size_t i = 0; i < COUNT; i++)
    {
        std::cout << data[i].angle << "; " << data[i].x << "; " << data[i].y << std::endl;
        circle.setPosition(sf::Vector2f(this->data[i].x - RADIUS, this->data[i].y - RADIUS));
        window->draw(circle);
    }

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
