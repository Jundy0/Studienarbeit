#include <iostream>

#include "lidarSim.h"

int main()
{
    std::cout << "Test" << std::endl;
    LidarSIM lidarSim = LidarSIM();

    lidarSim.setPWM(3);
    lidarSim.startScan();
    lidarSim.stopScan();
    lidarSim.getScanData((point_t *)0, 1000);
}
