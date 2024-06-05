#include "evasionAStar.h"

#include <queue>
#include <unordered_set>

#define INF 1e9

EvasionAStar::EvasionAStar(const std::shared_ptr<IVehicleActuator> &vehicleActuator)
    : EvasionControl(vehicleActuator)
{
}

void EvasionAStar::execute()
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
    fScore(ROUND(this->origin.x()), ROUND(this->origin.y())) = this->heuristic(this->origin, this->destination);
    parent(ROUND(this->origin.x()), ROUND(this->origin.y())) = {-1, -1};

    while (!openSet.empty())
    {
        Eigen::RowVector2d current = openSet.top().second;
        openSet.pop();

        if (current == this->destination)
        {
            while (current != Eigen::RowVector2d(-1, -1))
            {
                this->path.push_back(current);
                current = parent(ROUND(current.x()), ROUND(current.y()));
            }

            std::reverse(this->path.begin(), this->path.end());

            return;
        }

        std::vector<Eigen::RowVector2d> neighbors = {
            current + Eigen::RowVector2d(-1, 0),
            current + Eigen::RowVector2d(1, 0),
            current + Eigen::RowVector2d(0, -1),
            current + Eigen::RowVector2d(0, 1),
        };

        for (const auto &neighbor : neighbors)
        {
            size_t x = ROUND(neighbor.x());
            size_t y = ROUND(neighbor.y());

            if (x >= 0 && x < rows && y >= 0 && y < cols && this->isFree(x, y))
            {
                double tentativeGCost = gScore(ROUND(current.x()), ROUND(current.y())) + 1;

                if (tentativeGCost < gScore(x, y))
                {
                    parent(x, y) = current;
                    gScore(x, y) = tentativeGCost;
                    fScore(x, y) = gScore(x, y) + this->heuristic(neighbor, this->destination);
                    openSet.push({fScore(x, y), neighbor});
                }
            }
        }
    }
}

inline double EvasionAStar::heuristic(const Eigen::RowVector2d &p1, const Eigen::RowVector2d &p2)
{
    return std::abs(p1.x() - p2.x()) + std::abs(p1.y() - p2.y());
}
