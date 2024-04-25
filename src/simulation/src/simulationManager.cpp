#include "simulationManager.h"
#include "lidarSensorSim.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>

#define COUNT 720
#define RADIUS 5

SimulationManager::SimulationManager()
{
    this->window = new sf::RenderWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulation", sf::Style::Titlebar | sf::Style::Close);
    this->vehicle = new Vehicle(400.f, 400.f);

    Obstacle obstacle1 = Obstacle(100.f, 100.f, 50.f, 50.f);
    Obstacle obstacle2 = Obstacle(190.f, 170.f, 100.f, 50.f);
    Obstacle obstacle3 = Obstacle(500.f, 200.f, 100.f, 200.f);
    Obstacle obstacle4 = Obstacle(300.f, 600.f, 300.f, 50.f);

    this->obstacles.push_back(obstacle1);
    this->obstacles.push_back(obstacle2);
    this->obstacles.push_back(obstacle3);
    this->obstacles.push_back(obstacle4);

    this->lidarSensor = new LidarSensorSim(this->vehicle, this->obstacles);

    this->data = (lidar_point_t *)malloc(sizeof(lidar_point_t) * COUNT);
}

SimulationManager::~SimulationManager()
{
    delete this->window;
    delete this->vehicle;
    delete this->lidarSensor;

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
    window->clear(sf::Color::Yellow);

    for (auto &obstacle : this->obstacles)
    {
        window->draw(obstacle.getSprite());
    }

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

        circle.setPosition(circlePoint);
        window->draw(circle);

        sf::Vertex line[] =
            {
                sf::Vertex(vehiclePosition),
                sf::Vertex(intersectionPoint),
            };

        window->draw(line, 2, sf::Lines);
    }

    circle.setFillColor(sf::Color::Magenta);
    circle.setPosition(this->destination - sf::Vector2f(RADIUS, RADIUS));
    window->draw(circle);

    window->draw(this->vehicle->getSprite(collision));

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
            case sf::Keyboard::Key::W:
                this->vehicle->moveForward();
                break;
            case sf::Keyboard::Key::S:
                this->vehicle->moveBack();
                break;
            case sf::Keyboard::Key::A:
                this->vehicle->turnLeft();
                break;
            case sf::Keyboard::Key::D:
                this->vehicle->turnRight();
                break;
            case sf::Keyboard::Key::P:
                this->saveScanAsCsv();
                break;
            case sf::Keyboard::Key::Space:
                this->mode = this->mode == Mode::placeObstacle ? Mode::placeDestination : Mode::placeObstacle;
            }
            break;
        case sf::Event::KeyReleased:
            break;
        case sf::Event::MouseButtonPressed:
            switch (this->mode)
            {
            case Mode::placeObstacle:
                this->placingObstacle = true;
                this->newObstacleP1 = sf::Vector2f(this->ev.mouseButton.x, this->ev.mouseButton.y);
                break;
            case Mode::placeDestination:
                this->destination = sf::Vector2f(this->ev.mouseButton.x, this->ev.mouseButton.y);
                break;
            }
            break;
        case sf::Event::MouseButtonReleased:
            switch (this->mode)
            {
            case Mode::placeObstacle:
                if (this->placingObstacle)
                {
                    this->newObstacleP2 = sf::Vector2f(this->ev.mouseButton.x, this->ev.mouseButton.y);

                    const float startX = this->newObstacleP1.x <= this->newObstacleP2.x ? this->newObstacleP1.x : this->newObstacleP2.x;
                    const float startY = this->newObstacleP1.y <= this->newObstacleP2.y ? this->newObstacleP1.y : this->newObstacleP2.y;
                    const float length = std::abs(this->newObstacleP2.x - this->newObstacleP1.x);
                    const float height = std::abs(this->newObstacleP2.y - this->newObstacleP1.y);

                    this->obstacles.push_back(Obstacle(startX, startY, length, height));

                    this->placingObstacle = false;
                    this->newObstacleP1 = sf::Vector2f();
                    this->newObstacleP2 = sf::Vector2f();
                }
                break;
            }
            break;
        }
    }
}

void SimulationManager::saveScanAsCsv()
{
    // Create Directory for scans, if not exists
    std::string directoryName = "scans";
    if (!std::filesystem::exists(directoryName))
    {
        std::filesystem::create_directory(directoryName);
    }

    std::ofstream csvFile;
    std::stringstream counterString;
    counterString << std::setw(4) << std::setfill('0') << this->fileCount;
    std::string fileName = directoryName + "/scan" + counterString.str() + ".csv";

    csvFile.open(fileName);

    for (int i = 0; i < COUNT; i++)
    {
        csvFile << this->data[i].angle << "," << this->data[i].radius << std::endl;
    }

    csvFile.close();

    this->fileCount++;
}
