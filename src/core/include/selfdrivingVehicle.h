#ifndef __SELFDRIVING_VEHICLE_H__
#define __SELFDRIVING_VEHICLE_H__

#include "lidarSensor.h"
#include "vehicleActuator.h"

class SelfdrivingVehicle
{
public:
    SelfdrivingVehicle(ILidarSensor *lidarSensor, IVehicleActuator *vehicleControl);
    ~SelfdrivingVehicle();
};

#endif //__SELFDRIVING_VEHICLE_H__
