#ifndef __SELFDRIVING_VEHICLE_H__
#define __SELFDRIVING_VEHICLE_H__

#include "lidarSensor.h"
#include "vehicleActuator.h"
#include "particle.h"

#define SCAN_COUNT 360

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
    Particle particle;
    Eigen::MatrixX2d *currentScan;
    Eigen::MatrixX2d *lastScan;
    bool initial = true;

    lidar_point_t *lidarData;
};

#endif //__SELFDRIVING_VEHICLE_H__
