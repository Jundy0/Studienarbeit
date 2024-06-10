#include "evasionAdvancedAStar.h"

#include <queue>
#include <unordered_set>

#define INF 1e9

EvasionAdvancedAStar::EvasionAdvancedAStar(const std::shared_ptr<IVehicleActuator> &vehicleActuator)
    : EvasionControl(vehicleActuator)
{
}

void EvasionAdvancedAStar::execute()
{
    size_t rows = this->map->rows();
    size_t cols = this->map->cols();
    std::priority_queue<AdvancedAStarNode, std::vector<AdvancedAStarNode>, std::greater<AdvancedAStarNode>> openSet;

    Eigen::Matrix<AdvancedAStarNode, Eigen::Dynamic, Eigen::Dynamic> parent(rows, cols);

    AdvancedAStarNode startNode = AdvancedAStarNode(this->origin, this->direction, 0, this->heuristic(this->origin, this->destination));
    openSet.push(startNode);
    parent(ROUND(this->origin.x()), ROUND(this->origin.y())) = startNode;

    while (!openSet.empty())
    {
        AdvancedAStarNode currentNode = openSet.top();
        Eigen::RowVector2d currentPos = currentNode.position;
        double currentRot = currentNode.rotation;
        openSet.pop();

        if (currentPos == this->destination)
        {
            do
            {
                this->path.push_back(currentPos);
                currentPos = parent(ROUND(currentPos.x()), ROUND(currentPos.y())).position;
            } while (currentPos != this->origin);

            std::reverse(this->path.begin(), this->path.end());

            return;
        }

        std::vector<Eigen::RowVector2d> neighbors = {
            currentPos + Eigen::RowVector2d(-1, 0),
            currentPos + Eigen::RowVector2d(1, 0),
            currentPos + Eigen::RowVector2d(0, -1),
            currentPos + Eigen::RowVector2d(0, 1),
        };

        for (const auto &neighborPos : neighbors)
        {
            size_t x = ROUND(neighborPos.x());
            size_t y = ROUND(neighborPos.y());

            if (x >= 0 && x < rows && y >= 0 && y < cols && this->isFree(x, y))
            {
                double tentativeGCost = currentNode.gCost + 1;

                if (tentativeGCost < parent(x, y).gCost)
                {
                    AdvancedAStarNode neighborNode = AdvancedAStarNode(neighborPos, 0, tentativeGCost, this->heuristic(neighborPos, this->destination));
                    parent(x, y) = currentNode;
                    openSet.push(neighborNode);
                }
            }
        }
    }
}

inline double EvasionAdvancedAStar::heuristic(const Eigen::RowVector2d &p1, const Eigen::RowVector2d &p2)
{
    return std::abs(p1.x() - p2.x()) + std::abs(p1.y() - p2.y());
}
