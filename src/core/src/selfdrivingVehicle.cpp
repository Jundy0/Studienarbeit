#include "selfdrivingVehicle.h"
#include <iostream>

SelfdrivingVehicle::SelfdrivingVehicle(ILidarSensor *lidar, IVehicleActuator *vehicleControl)
{
    std::cout << "Evasion Control created" << std::endl;
}

SelfdrivingVehicle::~SelfdrivingVehicle()
{
}
