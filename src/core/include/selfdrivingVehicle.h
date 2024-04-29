#ifndef __SELFDRIVING_VEHICLE_H__
#define __SELFDRIVING_VEHICLE_H__

#include "lidarSensor.h"
#include "vehicleActuator.h"
#include "particle.h"
#include "slam.h"

#define SCAN_COUNT 360

class SelfdrivingVehicle
{
public:
    SelfdrivingVehicle(ILidarSensor *lidarSensor, IVehicleActuator *vehicleActuator);
    ~SelfdrivingVehicle();
    const lidar_point_t *getLidarDataPtr();
    const Eigen::MatrixXd *getGridMap();
    const Eigen::RowVector2d getPosition();
    const double getRotation();

    void update();

private:
    ILidarSensor *lidarSensor;
    IVehicleActuator *vehicleActuator;
    ISlam *slam;

    lidar_point_t *lidarData;
};

#endif //__SELFDRIVING_VEHICLE_H__
