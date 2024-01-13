#include "lidarSim.h"

#include <iostream>

LidarSIM::LidarSIM()
{
    std::cout << "LidarSIM: Constructor" << std::endl;
}

LidarSIM::~LidarSIM()
{
    std::cout << "LidarSIM: Destructor" << std::endl;
}

void LidarSIM::setPWM(int dutyCycle)
{
    std::cout << "LidarSIM: setPWM: " << dutyCycle << std::endl;
}

void LidarSIM::startScan()
{
    std::cout << "LidarSIM: startScan" << std::endl;
}

void LidarSIM::stopScan()
{
    std::cout << "LidarSIM: stopScan" << std::endl;
}

void LidarSIM::getScanData(point_t *data, size_t count)
{
    std::cout << "LidarSIM: getScanData: " << count << std::endl;
}
