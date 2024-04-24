#include "lidarSensorSim.h"

#include <iostream>
#include <cmath>
#include <cstring>

#include "intersection.h"

LidarSensorSim::LidarSensorSim(Vehicle *vehicle, std::vector<Obstacle> &obstacles)
{
    this->vehicle = vehicle;
    this->obstacles = &obstacles;
    std::cout << "LidarSensorSim: Constructor" << std::endl;
}

LidarSensorSim::~LidarSensorSim()
{
    std::cout << "LidarSensorSim: Destructor" << std::endl;
}

void LidarSensorSim::setPWM(int dutyCycle)
{
    std::cout << "LidarSensorSim: setPWM: " << dutyCycle << std::endl;
}

void LidarSensorSim::startScan()
{
    std::cout << "LidarSensorSim: startScan" << std::endl;
}

void LidarSensorSim::stopScan()
{
    std::cout << "LidarSensorSim: stopScan" << std::endl;
}

void LidarSensorSim::getScanData(lidar_point_t *data, size_t count)
{
    memset(data, 0, count * sizeof(lidar_point_t)); // clear data array

    const sf::FloatRect vehicleRect = this->vehicle->getPosition();
    const float vehicleRotation = this->vehicle->getRotation() * M_PI / 180.f;

    const float posX = vehicleRect.left + vehicleRect.width / 2;
    const float posY = vehicleRect.top + vehicleRect.height / 2;

    const sf::Vector2f vehiclePosition = sf::Vector2f(posX, posY);
    const sf::FloatRect windowRect = sf::FloatRect(0, 0, 800, 800);

    std::vector<sf::Vector2f> intersectionPoints = std::vector<sf::Vector2f>();

    for (size_t i = 0; i < count; i++)
    {
        const float radians = i * (360.f / count) * M_PI / 180.f;

        intersectsObstacles(vehiclePosition, radians + vehicleRotation, this->obstacles, windowRect, intersectionPoints);

        if (intersectionPoints.size() == 0)
        {
            std::cout << "Error: Ray did not intersect with obstacle or Window" << std::endl;
            continue;
        }

        data[i].radius = std::sqrt(std::pow(posX - intersectionPoints[0].x, 2) + std::pow(posY - intersectionPoints[0].y, 2));
        data[i].angle = radians;
        data[i].quality = 100.f;
        data[i].valid = true;
        data[i].x = intersectionPoints[0].x;
        data[i].y = intersectionPoints[0].y;

        intersectionPoints.clear();
    }
}
