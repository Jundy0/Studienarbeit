#include "selfdrivingVehicle.h"

SelfdrivingVehicle::SelfdrivingVehicle(ILidarSensor *lidarSensor, IVehicleActuator *vehicleActuator)
{
    this->lidarSensor = lidarSensor;
    this->vehicleActuator = vehicleActuator;
    this->lidarData = (lidar_point_t *)malloc(sizeof(lidar_point_t) * SCAN_COUNT);

    this->currentScan = new Eigen::MatrixX2d(SCAN_COUNT, 2);
    this->lastScan = new Eigen::MatrixX2d(SCAN_COUNT, 2);
}

SelfdrivingVehicle::~SelfdrivingVehicle()
{
    delete this->currentScan;
    delete this->lastScan;
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
    for (size_t i = 0; i < SCAN_COUNT; i++)
    {
        this->currentScan->row(i) = Eigen::RowVector2d{this->lidarData[i].angle, this->lidarData[i].radius};
    }

    if (this->initial)
    {
        this->initial = false;
    }
    else
    {
        this->particle.update(*this->lastScan, *this->currentScan);
    }

    Eigen::MatrixX2d *ptr = this->lastScan;
    this->lastScan = this->currentScan;
    this->currentScan = ptr;

    // ausweichen und actuator ansteuern

    this->vehicleActuator->update();
}
