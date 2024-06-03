#include "evasionControl.h"

#include <iostream>
#include <chrono>

EvasionControl::EvasionControl(const std::shared_ptr<IVehicleActuator> &vehicleActuator)
    : vehicleActuator(vehicleActuator)
{
}

void EvasionControl::setDestination(const Eigen::RowVector2d &destination)
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

void EvasionControl::update(const Eigen::MatrixXd *map, const Eigen::RowVector2d &position, double rotation)
{
    auto t1 = std::chrono::high_resolution_clock::now();

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

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Finished Pathfinding in " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms\n\n"
              << std::endl;

    this->printPath();

    // TODO: use VehicleActuator
}

bool EvasionControl::isFree(size_t x, size_t y)
{
    return (*this->map)(y, x) < PROB_OCC;
}

void EvasionControl::printPath()
{
    size_t pathLength = this->path.size();

    if (pathLength == 0)
    {
        std::cout << "No path exists between (" << this->origin.x() << ", " << this->origin.y() << ") and (" << this->destination.x() << ", " << this->destination.y() << ")" << std::endl;
    }
    else
    {
        std::cout << "Shortest path from (" << this->origin.x() << ", " << this->origin.y() << ") to (" << this->destination.x() << ", " << this->destination.y() << ") : ";
        for (int i = 0; i < pathLength; i++)
        {
            std::cout << "(" << this->path[i].x() << ", " << this->path[i].y() << ")";
            if (i != pathLength - 1)
                std::cout << " -> ";
        }
        std::cout << std::endl;
    }
}
