#include "evasionAdvancedAStar.h"

#include <queue>
#include <list>
#include <unordered_set>

#define INF 1e9
#define NEIGHBOR_DIFF 10

EvasionAdvancedAStar::EvasionAdvancedAStar(const std::shared_ptr<IVehicleActuator> &vehicleActuator)
    : EvasionControl(vehicleActuator)
{
}

void EvasionAdvancedAStar::execute()
{
    std::list<AdvancedAStarNode> nodes;
    size_t rows = this->map->rows();
    size_t cols = this->map->cols();
    auto cmp = [](const AdvancedAStarNode *left, const AdvancedAStarNode *right)
    {
        return *left > *right;
    };

    std::priority_queue<AdvancedAStarNode *, std::vector<AdvancedAStarNode *>, decltype(cmp)> openSet(cmp);

    nodes.emplace_back(this->origin, this->direction, 0, this->heuristic(this->origin, this->destination));
    AdvancedAStarNode *startNode = &nodes.back();
    startNode->parent = startNode;
    openSet.push(startNode);

    while (!openSet.empty())
    {
        AdvancedAStarNode *currentNode = openSet.top();
        openSet.pop();

        if (currentNode->position.x() >= this->destination.x() - NEIGHBOR_DIFF && currentNode->position.x() <= this->destination.x() + NEIGHBOR_DIFF && currentNode->position.y() >= this->destination.y() - NEIGHBOR_DIFF && currentNode->position.y() <= this->destination.y() + NEIGHBOR_DIFF)
        {
            do
            {
                this->path.push_back(currentNode->position);
                currentNode = currentNode->parent;
            } while (currentNode->position != this->origin);

            std::reverse(this->path.begin(), this->path.end());

            return;
        }

        std::vector<Eigen::RowVector2d> neighbors = this->getNeighbors(currentNode->position, currentNode->rotation);

        for (const auto &neighborPos : neighbors)
        {
            size_t x = ROUND(neighborPos.x());
            size_t y = ROUND(neighborPos.y());

            if (x >= 0 && x < rows && y >= 0 && y < cols && this->isFree(x, y))
            {
                double tentativeGCost = currentNode->gCost + 1;

                auto eq = [&](const AdvancedAStarNode &node)
                {
                    return neighborPos == node.position && tentativeGCost < node.gCost;
                };

                if (std::find_if(nodes.begin(), nodes.end(), eq) == nodes.end())
                {
                    Eigen::RowVector2d dirVec = currentNode->position - neighborPos;

                    double rotation = .0;
                    if (dirVec.x() == .0)
                    {
                        rotation = dirVec.y() > 0 ? M_PI / 2 : 3 * M_PI / 2;
                    }
                    else
                    {
                        rotation = std::atan2(dirVec.y(), dirVec.x()) + M_PI;
                    }

                    nodes.emplace_back(neighborPos, rotation, tentativeGCost, this->heuristic(neighborPos, this->destination), currentNode);
                    AdvancedAStarNode *neighborNode = &nodes.back();
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

std::vector<Eigen::RowVector2d> EvasionAdvancedAStar::getNeighbors(const Eigen::RowVector2d &position, const double rotation)
{
    std::vector<Eigen::RowVector2d> neighbors;

    const double rotLeft = rotation - VEHICLE_MAX_ROT_ANGLE;
    const double rotRight = rotation + VEHICLE_MAX_ROT_ANGLE;

    Eigen::Matrix2d rotMatStraight;
    rotMatStraight << std::cos(rotation), -std::sin(rotation), std::sin(rotation), std::cos(rotation);

    Eigen::Matrix2d rotMatLeft;
    rotMatLeft << std::cos(rotLeft), -std::sin(rotLeft), std::sin(rotLeft), std::cos(rotLeft);

    Eigen::Matrix2d rotMatRight;
    rotMatRight << std::cos(rotRight), -std::sin(rotRight), std::sin(rotRight), std::cos(rotRight);

    const Eigen::RowVector2d directionForeward(NEIGHBOR_DIFF, 0);
    const Eigen::RowVector2d directionBackward(-NEIGHBOR_DIFF, 0);

    neighbors.push_back(position + (directionForeward * rotMatStraight));
    neighbors.push_back(position + (directionForeward * rotMatLeft));
    neighbors.push_back(position + (directionForeward * rotMatRight));
    // neighbors.push_back(position + (directionBackward * rotMatStraight));
    // neighbors.push_back(position + (directionBackward * rotMatLeft));
    // neighbors.push_back(position + (directionBackward * rotMatRight));

    return neighbors;
}
