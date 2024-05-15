#include "selfdrivingVehicle.h"
#include "slamHandler.h"

SelfdrivingVehicle::SelfdrivingVehicle(ILidarSensor *lidarSensor, IVehicleActuator *vehicleActuator)
{
    this->lidarSensor = lidarSensor;
    this->vehicleActuator = vehicleActuator;
    this->slam = new SlamHandler(SCAN_COUNT);
    this->lidarData = (lidar_point_t *)malloc(sizeof(lidar_point_t) * SCAN_COUNT);
    this->evasionControl = new EvasionControl(this->vehicleActuator);
    this->evasionControl->setDestination(Eigen::RowVector2d(500, 500));
}

SelfdrivingVehicle::~SelfdrivingVehicle()
{
    delete this->slam;
    free(this->lidarData);
    delete this->evasionControl;
}

const lidar_point_t *SelfdrivingVehicle::getLidarDataPtr()
{
    return this->lidarData;
}

const Eigen::MatrixXd *SelfdrivingVehicle::getGridMap()
{
    return this->slam->getGridMap();
}

const Eigen::RowVector2d SelfdrivingVehicle::getPosition()
{
    return this->slam->getPosition();
}

const double SelfdrivingVehicle::getRotation()
{
    return this->slam->getRotation();
}

const std::vector<Eigen::RowVector2d> SelfdrivingVehicle::getPath()
{
    return this->evasionControl->getPath();
}

void SelfdrivingVehicle::update()
{
    // get lidar Data
    this->lidarSensor->getScanData(this->lidarData, SCAN_COUNT);

    const std::pair<Eigen::RowVector2d, double> odometry = this->vehicleActuator->getOdometry();

    // execute SLAM
    this->slam->update(this->lidarData, odometry.first, odometry.second);

    // Evation Control and set values of actuator
    this->evasionControl->update(this->getGridMap(), this->getPosition());

    // Update Actuator
    this->vehicleActuator->update();
}
