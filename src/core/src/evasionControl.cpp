#include "evasionControl.h"

#include <iostream>
#include <chrono>
#include <cmath>

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

void EvasionControl::update(std::shared_ptr<Eigen::MatrixXd> map, const Eigen::RowVector2d &position, double rotation)
{
    auto t1 = std::chrono::high_resolution_clock::now();

    this->map = map;
    this->origin = Eigen::RowVector2d(std::round(position.x() / (MAP_WIDTH / GRID_WIDTH)), std::round(position.y() / (MAP_HEIGHT / GRID_HEIGHT)));
    this->direction = rotation;

    this->infalteObstacles();

    this->path.clear();

    // check if destination is set
    if (destination.x() == 0 && destination.y() == 0)
    {
        std::cout << "No Destination set" << std::endl;
        return;
    }

    // check if destination is reached
    if (origin == destination)
    {
        std::cout << "Reached Destination" << std::endl;
        return;
    }

    const double destVal = (*this->map)(ROUND(this->destination.y()), ROUND(this->destination.x()));

    if (destVal >= PROB_OCC)
    {
        std::cout << "Destination inside Obsatcle" << std::endl;
        return;
    }

    if (destVal >= INFLATED)
    {
        std::cout << "Destination to close to Obstacle" << std::endl;
        return;
    }

    // execute Pathfinding Algorithm
    this->execute();

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Finished Pathfinding in " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms\n"
              << std::endl;

    this->printPath();

    // TODO: use VehicleActuator
}

bool EvasionControl::isFree(size_t x, size_t y)
{
    return (*this->map)(y, x) < INFLATED;
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
        std::cout << "\n\n"
                  << std::endl;
    }
}

void EvasionControl::infalteObstacles()
{
    const size_t rows = this->map->rows();
    const size_t cols = this->map->cols();

    for (size_t i = 0; i < rows; ++i)
    {
        for (size_t j = 0; j < cols; ++j)
        {
            if ((*this->map)(i, j) >= PROB_OCC)
            {
                for (int x = -VEHICLE_RADIUS; x <= VEHICLE_RADIUS; ++x)
                {
                    // TODO: better for loop value
                    const int circleValue = std::round(VEHICLE_RADIUS * std::cos(std::abs(x) * M_PI / (2 * VEHICLE_RADIUS)));
                    for (int y = -circleValue; y <= circleValue; ++y)
                    {
                        int nx = i + x;
                        int ny = j + y;
                        if (nx >= 0 && ny >= 0 && nx < rows && ny < cols && (*this->map)(nx, ny) < PROB_OCC)
                        {
                            (*this->map)(nx, ny) = INFLATED;
                        }
                    }
                }
            }
        }
    }
}
