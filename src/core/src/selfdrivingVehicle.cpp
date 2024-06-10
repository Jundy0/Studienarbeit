#include "selfdrivingVehicle.h"
#include "slamHandler.h"
#include "evasionBasicAStar.h"
#include "evasionAdvancedAStar.h"
#include "evasionThetaStar.h"

SelfdrivingVehicle::SelfdrivingVehicle(const std::shared_ptr<ILidarSensor> &lidarSensor, const std::shared_ptr<IVehicleActuator> &vehicleActuator)
    : lidarSensor(lidarSensor),
      vehicleActuator(vehicleActuator)
{
    this->slam = std::make_unique<SlamHandler>(SCAN_COUNT);
    this->lidarData = (lidar_point_t *)malloc(sizeof(lidar_point_t) * SCAN_COUNT);
    // this->evasionControl = std::make_unique<EvasionBasicAStar>(this->vehicleActuator);
    this->evasionControl = std::make_unique<EvasionAdvancedAStar>(this->vehicleActuator);
    // this->evasionControl = std::make_unique<EvasionThetaStar>(this->vehicleActuator);

    this->lastTime = std::chrono::high_resolution_clock::now();
}

SelfdrivingVehicle::~SelfdrivingVehicle()
{
    free(this->lidarData);
}

const lidar_point_t *SelfdrivingVehicle::getLidarDataPtr()
{
    return this->lidarData;
}

std::shared_ptr<Eigen::MatrixXd> SelfdrivingVehicle::getGridMap()
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
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - this->lastTime).count();

    if (timeDiff > 200)
    {
        this->lastTime = currentTime;

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
