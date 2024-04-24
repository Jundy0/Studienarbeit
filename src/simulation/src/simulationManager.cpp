#include "simulationManager.h"
#include "lidarSensorSim.h"

#include <iostream>

#define COUNT 4
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
    this->collision = false;
    this->pollEvent();

    this->vehicle->update();

    // Check for collisions
    for (auto &obstacle : this->obstacles)
    {
        if (obstacle.getPosition().intersects(this->vehicle->getPosition()))
        {
            this->collision = true;
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

    window->draw(this->vehicle->getShape(collision));

    sf::CircleShape circle = sf::CircleShape(RADIUS);
    circle.setFillColor(sf::Color::Red);

    const sf::FloatRect vehicleRect = this->vehicle->getPosition();

    const float posX = vehicleRect.left + vehicleRect.width / 2;
    const float posY = vehicleRect.top + vehicleRect.height / 2;

    const sf::Vector2f vehiclePosition = sf::Vector2f(posX, posY);

    for (size_t i = 0; i < COUNT; i++)
    {
        const sf::Vector2f intersectionPoint = sf::Vector2f(this->data[i].x, this->data[i].y);
        const sf::Vector2f circlePoint = intersectionPoint - sf::Vector2f(RADIUS, RADIUS); // adapt circle, so that the center is at the intersection Point

        std::cout << data[i].angle << "; " << data[i].x << "; " << data[i].y << std::endl;
        circle.setPosition(circlePoint);
        window->draw(circle);

        sf::Vertex line[] =
            {
                sf::Vertex(vehiclePosition),
                sf::Vertex(intersectionPoint),
            };

        window->draw(line, 2, sf::Lines);
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
