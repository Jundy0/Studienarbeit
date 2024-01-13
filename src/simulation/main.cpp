#include <iostream>

#include "lidarSim.h"
#include "vehicleControlSim.h"
#include "evasionControl.h"

int main()
{
    std::cout << "Test" << std::endl;
    LidarSIM lidarSim = LidarSIM();

    lidarSim.setPWM(3);
    lidarSim.startScan();
    lidarSim.stopScan();
    lidarSim.getScanData((point_t *)0, 1000);

    VehicleControlSim vehicleControlSim = VehicleControlSim();

    EvasionControl evasionControl = EvasionControl(&lidarSim, &vehicleControlSim);
}
