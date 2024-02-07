#ifndef __SELFDRIVING_VEHICLE_H__
#define __SELFDRIVING_VEHICLE_H__

#include "lidar.h"
#include "vehicleControl.h"

class SelfdrivingVehicle
{
public:
    SelfdrivingVehicle(ILidarSensor *lidarSensor, IVehicleActuator *vehicleControl);
    ~SelfdrivingVehicle();
};

#endif //__SELFDRIVING_VEHICLE_H__
