#ifndef __SELFDRIVING_VEHICLE_H__
#define __SELFDRIVING_VEHICLE_H__

#include "lidarSensor.h"
#include "vehicleActuator.h"

class SelfdrivingVehicle
{
public:
    SelfdrivingVehicle(ILidarSensor *lidarSensor, IVehicleActuator *vehicleActuator);
    ~SelfdrivingVehicle();
    const lidar_point_t *getLidarDataPtr();

    void update();

private:
    ILidarSensor *lidarSensor;
    IVehicleActuator *vehicleActuator;

    lidar_point_t *lidarData;
};

#endif //__SELFDRIVING_VEHICLE_H__
