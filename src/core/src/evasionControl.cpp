#include "evasionControl.h"

#include "icpSettings.h"

#include <iostream>
#include <queue>

#define ROUND(x) (size_t) std::round(x)

EvasionControl::EvasionControl(IVehicleActuator *vehicleActuator)
{
    this->vehicleActuator = vehicleActuator;
}

EvasionControl::~EvasionControl()
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

void EvasionControl::update(const Eigen::MatrixXd *map, Eigen::RowVector2d position)
{
    this->map = map;
    this->origin = Eigen::RowVector2d(std::round(position.x() / (MAP_WIDTH / GRID_WIDTH)), std::round(position.y() / (MAP_HEIGHT / GRID_HEIGHT)));

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

    this->AStar();
}

void EvasionControl::AStar()
{
    size_t rows = this->map->rows();
    size_t cols = this->map->cols();
    auto cmp = [](std::pair<double, Eigen::RowVector2d> left, std::pair<double, Eigen::RowVector2d> right)
    { return left.first > right.first; };
    std::priority_queue<std::pair<double, Eigen::RowVector2d>, std::vector<std::pair<double, Eigen::RowVector2d>>, decltype(cmp)> openSet(cmp);

    Eigen::MatrixXd gScore = Eigen::MatrixXd::Constant(rows, cols, INF);
    Eigen::MatrixXd fScore = Eigen::MatrixXd::Constant(rows, cols, INF);
    Eigen::Matrix<Eigen::RowVector2d, Eigen::Dynamic, Eigen::Dynamic> parent(rows, cols);

    openSet.push({0, this->origin});
    gScore(ROUND(this->origin.x()), ROUND(this->origin.y())) = 0;
    fScore(ROUND(this->origin.x()), ROUND(this->origin.y())) = heuristic(this->origin);
    parent(ROUND(this->origin.x()), ROUND(this->origin.y())) = {-1, -1};

    while (!openSet.empty())
    {
        Eigen::RowVector2d current = openSet.top().second;
        openSet.pop();

        if (current == this->destination)
        {
            // Path found, now trace back the path using parent array
            std::vector<Eigen::RowVector2d> path;
            while (current != Eigen::RowVector2d(-1, -1))
            {
                path.push_back(current);
                current = parent(ROUND(current.x()), ROUND(current.y()));
            }

            // Print the path in reverse order
            std::cout << "Shortest path from (" << this->origin.x() << ", " << this->origin.y() << ") to (" << this->destination.x() << ", " << this->destination.y() << ") : ";
            for (int i = path.size() - 1; i >= 0; i--)
            {
                std::cout << "(" << path[i].x() << ", " << path[i].y() << ")";
                if (i != 0)
                    std::cout << " -> ";
            }
            std::cout << std::endl;

            this->path = path;
            return;
        }

        std::vector<Eigen::RowVector2d> neighbors = {current + Eigen::RowVector2d(-1, 0), current + Eigen::RowVector2d(1, 0), current + Eigen::RowVector2d(0, -1), current + Eigen::RowVector2d(0, 1)};
        for (const auto &neighbor : neighbors)
        {
            size_t x = ROUND(neighbor.x());
            size_t y = ROUND(neighbor.y());

            if (x >= 0 && x < rows && y >= 0 && y < cols && (*this->map)(x, y) <= 0)
            {
                double tentative_g_cost = gScore(ROUND(current.x()), ROUND(current.y())) + 1;

                if (tentative_g_cost < gScore(x, y))
                {
                    parent(x, y) = current;
                    gScore(x, y) = tentative_g_cost;
                    fScore(x, y) = gScore(x, y) + heuristic(neighbor);
                    openSet.push({fScore(x, y), neighbor});
                }
            }
        }
    }

    std::cout << "No path exists between (" << this->origin.x() << ", " << this->origin.y() << ") and (" << this->destination.x() << ", " << this->destination.y() << ")" << std::endl;
}

double EvasionControl::heuristic(Eigen::RowVector2d p)
{
    return std::abs(p.x() - this->destination.x()) + std::abs(p.y() - this->destination.y());
}
