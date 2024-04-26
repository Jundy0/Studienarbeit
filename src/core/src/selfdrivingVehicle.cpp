#include "selfdrivingVehicle.h"
#include <iostream>

#define SCAN_COUNT 720

SelfdrivingVehicle::SelfdrivingVehicle(ILidarSensor *lidarSensor, IVehicleActuator *vehicleActuator)
{
    this->lidarSensor = lidarSensor;
    this->vehicleActuator = vehicleActuator;
    this->lidarData = (lidar_point_t *)malloc(sizeof(lidar_point_t) * SCAN_COUNT);
    std::cout << "Evasion Control created" << std::endl;
}

SelfdrivingVehicle::~SelfdrivingVehicle()
{
    free(this->lidarData);
}

const lidar_point_t *SelfdrivingVehicle::getLidarDataPtr()
{
    return this->lidarData;
}

void SelfdrivingVehicle::update()
{
    this->lidarSensor->getScanData(lidarData, SCAN_COUNT);

    // icp oder so

    // ausweichen und actuator ansteuern

    this->vehicleActuator->update();
}
