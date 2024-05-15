#include "evasionControl.h"

#include "icpSettings.h"

#include <iostream>
#include <queue>

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
    int rows = this->map->rows();
    int cols = this->map->cols();
    auto cmp = [](std::pair<double, Eigen::RowVector2d> left, std::pair<double, Eigen::RowVector2d> right)
    { return left.first > right.first; };
    std::priority_queue<std::pair<double, Eigen::RowVector2d>, std::vector<std::pair<double, Eigen::RowVector2d>>, decltype(cmp)> openSet(cmp);

    Eigen::MatrixXd g_cost = Eigen::MatrixXd::Constant(rows, cols, INF);
    Eigen::MatrixXd f_cost = Eigen::MatrixXd::Constant(rows, cols, INF);
    Eigen::Matrix<Eigen::RowVector2d, Eigen::Dynamic, Eigen::Dynamic> parent(rows, cols);

    openSet.push({0, this->origin});
    g_cost((int)std::round(this->origin.x()), (int)std::round(this->origin.y())) = 0;
    f_cost((int)std::round(this->origin.x()), (int)std::round(this->origin.y())) = heuristic(this->origin, this->destination);
    parent((int)std::round(this->origin.x()), (int)std::round(this->origin.y())) = {-1, -1};

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
                current = parent((int)std::round(current.x()), (int)std::round(current.y()));
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
            int x = (int)std::round(neighbor.x());
            int y = (int)std::round(neighbor.y());

            if (x >= 0 && x < rows && y >= 0 && y < cols && (*this->map)(x, y) <= 0)
            {
                double tentative_g_cost = g_cost((int)std::round(current.x()), (int)std::round(current.y())) + 1;

                if (tentative_g_cost < g_cost(x, y))
                {
                    parent(x, y) = current;
                    g_cost(x, y) = tentative_g_cost;
                    f_cost(x, y) = g_cost(x, y) + heuristic(neighbor, this->destination);
                    openSet.push({f_cost(x, y), neighbor});
                }
            }
        }
    }

    std::cout << "No path exists between (" << this->origin.x() << ", " << this->origin.y() << ") and (" << this->destination.x() << ", " << this->destination.y() << ")" << std::endl;
}

double EvasionControl::heuristic(Eigen::RowVector2d v1, Eigen::RowVector2d v2)
{
    return std::abs(v1.x() - v2.x()) + std::abs(v1.y() - v2.y());
}
