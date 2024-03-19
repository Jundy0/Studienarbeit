#include "lidarSensorSim.h"

#include <iostream>
#include <math.h>

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
    const float posX = this->vehicle->getPosition().left;
    const float posY = this->vehicle->getPosition().top;

    const sf::Vector2f vehiclePosition = sf::Vector2f(posX, posY);
    const sf::FloatRect windowRect = sf::FloatRect(0, 0, 800, 800);

    std::vector<sf::Vector2f> intersectionPoints = std::vector<sf::Vector2f>();

    for (size_t i = 0; i < count; i++)
    {
        float radians = i * M_PI / 180.0;

        intersectsObstacles(vehiclePosition, radians, this->obstacles, windowRect, intersectionPoints);

        data[i].radius = -1;
        data[i].angle = i;
        data[i].quality = 100.f;
        data[i].valid = true;
        data[i].x = intersectionPoints[0].x;
        data[i].y = intersectionPoints[0].y;

        intersectionPoints.clear();
    }
}
