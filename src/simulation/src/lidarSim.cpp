#include "lidarSensorSim.h"

#include <iostream>

LidarSensorSim::LidarSensorSim()
{
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
    std::cout << "LidarSensorSim: getScanData: " << count << std::endl;
}
