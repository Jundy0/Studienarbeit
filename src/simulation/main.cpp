#include <iostream>

#include "lidarSensorSim.h"
#include "vehicleActuatorSim.h"
#include "selfdrivingVehicle.h"
#include "simulationManager.h"

int main(int argc, char **argv)
{
    std::cout << "Test" << std::endl;
    LidarSensorSim lidarSensorSim = LidarSensorSim();

    lidarSensorSim.setPWM(3);
    lidarSensorSim.startScan();
    lidarSensorSim.stopScan();
    lidarSensorSim.getScanData((lidar_point_t *)0, 1000);

    VehicleActuatorSim vehicleActuatorSim = VehicleActuatorSim();

    SelfdrivingVehicle selfdrivingVehicle = SelfdrivingVehicle(&lidarSensorSim, &vehicleActuatorSim);

    SimulationManager simulationManager;
    simulationManager.run();
}
