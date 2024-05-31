#include "evasionControl.h"

EvasionControl::EvasionControl(const std::shared_ptr<IVehicleActuator> &vehicleActuator)
    : vehicleActuator(vehicleActuator)
{
}

void EvasionControl::setDestination(Eigen::RowVector2d destination)
{
    this->destination = Eigen::RowVector2d(std::round(destination.x() / (MAP_WIDTH / GRID_WIDTH)), std::round(destination.y() / (MAP_HEIGHT / GRID_HEIGHT)));
}

const Eigen::RowVector2d EvasionControl::getDestination()
{
    return this->destination;
}

const std::vector<Eigen::RowVector2d> EvasionControl::getPath()
{
    return this->path;
}

void EvasionControl::update(const Eigen::MatrixXd *map, Eigen::RowVector2d position, double rotation)
{
    this->map = map;
    this->origin = Eigen::RowVector2d(std::round(position.x() / (MAP_WIDTH / GRID_WIDTH)), std::round(position.y() / (MAP_HEIGHT / GRID_HEIGHT)));
    this->direction = rotation;

    // check if destination is set
    if (destination.x() == 0 && destination.y() == 0)
    {
        return;
    }

    // check if destination is reached
    if (origin == destination)
    {
        return;
    }

    // execute Pathfinding Algorithm
    this->execute();

    // TODO: use VehicleActuator
}
