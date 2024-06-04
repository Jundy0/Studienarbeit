#include "controlWindow.h"

#include <fstream>
#include <filesystem>
#include <iostream>

ControlWindow::ControlWindow(const std::shared_ptr<ILidarSensor> &lidarSensor, const std::shared_ptr<IVehicleActuator> &vehicleActuator, const std::shared_ptr<SelfdrivingVehicle> &selfdrivingVehicle, const std::shared_ptr<Vehicle> &vehicle, std::vector<Obstacle> &obstacles)
    : lidarSensor(lidarSensor),
      vehicleActuator(vehicleActuator),
      selfdrivingVehicle(selfdrivingVehicle),
      vehicle(vehicle),
      obstacles(&obstacles)
{
    this->window = std::make_unique<sf::RenderWindow>(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulation", sf::Style::Titlebar | sf::Style::Close);
    this->window->setFramerateLimit(60);
    this->font.loadFromFile("../res/Arial.ttf");
    this->fpsDisplay.setFont(this->font);
    this->fpsDisplay.setCharacterSize(30);
    this->fpsDisplay.setFillColor(sf::Color::Red);
}

ControlWindow::~ControlWindow()
{
}

void ControlWindow::update()
{
    this->collision = false;
    this->pollEvent();

    this->selfdrivingVehicle->update();

    // Check for collisions
    for (auto &obstacle : *this->obstacles)
    {
        if (obstacle.getPosition().intersects(this->vehicle->getPosition()))
        {
            this->collision = true;
            std::cout << "Error: Vehicle crashed into an obstacle!" << std::endl;
        }
    }

    const float elapsedTime = this->clock.restart().asSeconds();
    this->fps = 1.f / elapsedTime;
}

void ControlWindow::render()
{
    this->window->clear(sf::Color::Yellow);

    for (auto &obstacle : *this->obstacles)
    {
        this->window->draw(obstacle.getSprite());
    }

    sf::CircleShape circle = sf::CircleShape(CIRCLE_RADIUS);
    circle.setFillColor(sf::Color::Red);

    const sf::FloatRect vehicleRect = this->vehicle->getPosition();

    const float posX = vehicleRect.left + vehicleRect.width / 2;
    const float posY = vehicleRect.top + vehicleRect.height / 2;

    const sf::Vector2f vehiclePosition = sf::Vector2f(posX, posY);

    const lidar_point_t *lidarData = this->selfdrivingVehicle->getLidarDataPtr();

    for (size_t i = 0; i < SCAN_COUNT; i++)
    {
        const sf::Vector2f intersectionPoint = sf::Vector2f(lidarData[i].x, lidarData[i].y);
        const sf::Vector2f circlePoint = intersectionPoint - sf::Vector2f(CIRCLE_RADIUS, CIRCLE_RADIUS); // adapt circle, so that the center is at the intersection Point

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
        circle.setPosition(this->destination - sf::Vector2f(CIRCLE_RADIUS, CIRCLE_RADIUS));
        window->draw(circle);
    }

    if (this->placingObstacle) // Helper point for Placing new Obstacle
    {
        circle.setFillColor(sf::Color::Green);
        circle.setPosition(this->newObstacleP1 - sf::Vector2f(CIRCLE_RADIUS, CIRCLE_RADIUS));
        window->draw(circle);
    }

    window->draw(this->vehicle->getSprite(collision));

    this->fpsDisplay.setString(std::to_string(this->fps));
    window->draw(this->fpsDisplay);

    window->display();
}

bool ControlWindow::isOpen()
{
    return this->window->isOpen();
}

void ControlWindow::close()
{
    this->window->close();
}

void ControlWindow::pollEvent()
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
                // TODO Check if Destination is in infalted Obstacle
                sf::Vector2f newDestination = sf::Vector2f(this->ev.mouseButton.x, this->ev.mouseButton.y);
                // Check if new Destination is in obstacle
                bool contains = false;
                for (auto &obstacle : *this->obstacles)
                {
                    if (obstacle.getPosition().contains(newDestination))
                    {
                        contains = true;
                    }
                }
                if (!contains)
                {
                    this->destination = newDestination;
                    this->selfdrivingVehicle->setDestination(Eigen::RowVector2d(newDestination.x + (MAP_WIDTH / 2) - (WINDOW_WIDTH / 2), newDestination.y + (MAP_HEIGHT / 2) - (WINDOW_HEIGHT / 2)));
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
                        this->obstacles->push_back(newObstacle);
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
}

void ControlWindow::saveScanAsCsv()
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

    const lidar_point_t *lidarData = this->selfdrivingVehicle->getLidarDataPtr();

    for (int i = 0; i < SCAN_COUNT; i++)
    {
        csvFile << lidarData[i].angle << "," << lidarData[i].radius << std::endl;
    }

    csvFile.close();

    this->fileCount++;
}
