#include "simulationManager.h"
#include "lidarSensorSim.h"
#include "vehicleActuatorSim.h"

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
    this->window2 = new sf::RenderWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulation 2", sf::Style::Titlebar | sf::Style::Close);
    this->window2Image.create(WINDOW_WIDTH, WINDOW_HEIGHT, sf::Color::Transparent);
    this->window2Texture.loadFromImage(this->window2Image);
    this->window2Sprite.setTexture(this->window2Texture, true);

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
    this->vehicleActuator = new VehicleActuatorSim(this->vehicle);

    this->data = (lidar_point_t *)malloc(sizeof(lidar_point_t) * COUNT);
}

SimulationManager::~SimulationManager()
{
    delete this->window;
    delete this->window2;
    delete this->vehicle;
    delete this->lidarSensor;

    free(this->data);
}

void SimulationManager::run()
{
    this->window->setFramerateLimit(60);

    while (this->window->isOpen() && this->window2->isOpen())
    {
        this->update();
        this->render();
    }

    if (this->window->isOpen())
    {
        this->window->close();
    }

    if (this->window2->isOpen())
    {
        this->window2->close();
    }
}

void SimulationManager::update()
{
    this->collision = false;
    this->pollEvent();

    this->vehicleActuator->update();

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

    if (this->destination != sf::Vector2f(0, 0)) // Default value for Destaination Point
    {
        circle.setFillColor(sf::Color::Magenta);
        circle.setPosition(this->destination - sf::Vector2f(RADIUS, RADIUS));
        window->draw(circle);
    }

    if (this->placingObstacle) // Helper point for Placing new Obstacle
    {
        circle.setFillColor(sf::Color::Green);
        circle.setPosition(this->newObstacleP1 - sf::Vector2f(RADIUS, RADIUS));
        window->draw(circle);
    }

    window->draw(this->vehicle->getSprite(collision));

    window->display();

    // windows 2
    window2->clear(sf::Color::Yellow);

    // for (size_t i = 0; i < WINDOW_WIDTH; i++)
    // {
    //     for (size_t j = 0; j < WINDOW_HEIGHT; j++)
    //     {
    //         window2Image.setPixel(i, j, sf::Color::Transparent);
    //     }
    // }

    for (size_t i = 0; i < COUNT; i++)
    {
        window2Image.setPixel(this->data[i].x, this->data[i].y, sf::Color::Red);
    }

    window2Texture.loadFromImage(window2Image);

    window2->draw(this->window2Sprite);

    window2->display();
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
                this->vehicleActuator->setForeward(1.0);
                break;
            case sf::Keyboard::Key::S:
                this->vehicleActuator->setBackward(1.0);
                break;
            case sf::Keyboard::Key::A:
                this->vehicleActuator->setLeft(1.0);
                break;
            case sf::Keyboard::Key::D:
                this->vehicleActuator->setRight(1.0);
                break;
            case sf::Keyboard::Key::P:
                this->saveScanAsCsv();
                break;
            case sf::Keyboard::Key::Space:
                this->mode = this->mode == Mode::placeObstacle ? Mode::placeDestination : Mode::placeObstacle;
            }
            break;
        case sf::Event::KeyReleased:
            switch (this->ev.key.code)
            {
            case sf::Keyboard::Key::W:
                this->vehicleActuator->setForeward(.0);
                break;
            case sf::Keyboard::Key::S:
                this->vehicleActuator->setBackward(.0);
                break;
            case sf::Keyboard::Key::A:
                this->vehicleActuator->setLeft(.0);
                break;
            case sf::Keyboard::Key::D:
                this->vehicleActuator->setRight(.0);
                break;
            }
            break;
        case sf::Event::MouseButtonPressed:
            switch (this->mode)
            {
            case Mode::placeObstacle:
                this->placingObstacle = true;
                this->newObstacleP1 = sf::Vector2f(this->ev.mouseButton.x, this->ev.mouseButton.y);
                break;
            case Mode::placeDestination:
                sf::Vector2f newDestination = sf::Vector2f(this->ev.mouseButton.x, this->ev.mouseButton.y);
                // Check if new Destination is in obstacle
                bool contains = false;
                for (auto &obstacle : this->obstacles)
                {
                    if (obstacle.getPosition().contains(newDestination))
                    {
                        contains = true;
                    }
                }
                if (!contains)
                {
                    this->destination = newDestination;
                }
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

                    Obstacle newObstacle = Obstacle(startX, startY, length, height);

                    sf::FloatRect newObstacleRect = newObstacle.getPosition();
                    if (!newObstacleRect.intersects(this->vehicle->getPosition()) && !newObstacleRect.contains(this->destination)) // New Obstacle collide with vehicle or destination Point
                    {
                        this->obstacles.push_back(newObstacle);
                    }

                    this->placingObstacle = false;
                    this->newObstacleP1 = sf::Vector2f();
                    this->newObstacleP2 = sf::Vector2f();
                }
                break;
            }
            break;
        }
    }

    // window 2
    while (this->window2->pollEvent(this->ev))
    {
        switch (this->ev.type)
        {
        case sf::Event::Closed:
            this->window2->close();
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
