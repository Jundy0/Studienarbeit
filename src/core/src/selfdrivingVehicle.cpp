#include "selfdrivingVehicle.h"
#include "slamHandler.h"
#include "evasionAStar.h"
#include "evasionThetaStar.h"

SelfdrivingVehicle::SelfdrivingVehicle(ILidarSensor *lidarSensor, IVehicleActuator *vehicleActuator)
{
    this->lidarSensor = lidarSensor;
    this->vehicleActuator = vehicleActuator;
    this->slam = new SlamHandler(SCAN_COUNT);
    this->lidarData = (lidar_point_t *)malloc(sizeof(lidar_point_t) * SCAN_COUNT);
    this->evasionControl = new EvasionAStar(this->vehicleActuator);
    this->evasionControl->setDestination(Eigen::RowVector2d(2450, 2450));

    this->frameCount = 0;
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

const Eigen::RowVector2d SelfdrivingVehicle::getDestination()
{
    return this->evasionControl->getDestination();
}

const std::vector<Eigen::RowVector2d> SelfdrivingVehicle::getPath()
{
    return this->evasionControl->getPath();
}

void SelfdrivingVehicle::setDestination(Eigen::RowVector2d destination)
{
    this->evasionControl->setDestination(destination);
}

void SelfdrivingVehicle::update()
{
    this->frameCount++;
    this->frameCount = this->frameCount % 10;

    if (this->frameCount == 0)
    {
        // get lidar Data
        this->lidarSensor->getScanData(this->lidarData, SCAN_COUNT);

        const std::pair<Eigen::RowVector2d, double> odometry = this->vehicleActuator->getOdometry();

        // execute SLAM
        this->slam->update(this->lidarData, odometry.first, odometry.second);

        // Evation Control and set values of actuator
        this->evasionControl->update(this->getGridMap(), this->getPosition(), this->getRotation());
    }
    // Update Actuator
    this->vehicleActuator->update();
}
