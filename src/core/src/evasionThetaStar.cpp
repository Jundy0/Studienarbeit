#include "evasionThetaStar.h"

#include <queue>
#include <unordered_set>

EvasionThetaStar::EvasionThetaStar(const std::shared_ptr<IVehicleActuator> &vehicleActuator)
    : EvasionControl(vehicleActuator)
{
}

void EvasionThetaStar::execute()
{
    size_t rows = this->map->rows();
    size_t cols = this->map->cols();

    std::priority_queue<ThetaStarNode, std::vector<ThetaStarNode>, std::greater<ThetaStarNode>> openSet;
    std::unordered_set<long> closedSet;

    auto hash = [](const Eigen::RowVector2d &v)
    { return std::hash<double>()(v.x()) ^ std::hash<double>()(v.y()); };

    openSet.emplace(this->origin, 0.0, heuristic(this->origin, this->destination));

    while (!openSet.empty())
    {
        ThetaStarNode current = openSet.top();
        openSet.pop();

        if (current.position == this->destination)
        {
            std::vector<Eigen::RowVector2d> path;
            ThetaStarNode *node = &current;
            while (node != nullptr)
            {
                path.push_back(node->position);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            this->path = path;

            return;
        }

        closedSet.insert(hash(current.position));

        std::vector<Eigen::RowVector2d> neighbors = {
            current.position + Eigen::RowVector2d(-1, 0),
            current.position + Eigen::RowVector2d(1, 0),
            current.position + Eigen::RowVector2d(0, -1),
            current.position + Eigen::RowVector2d(0, 1),
            current.position + Eigen::RowVector2d(-1, -1),
            current.position + Eigen::RowVector2d(-1, 1),
            current.position + Eigen::RowVector2d(1, -1),
            current.position + Eigen::RowVector2d(1, 1),
        };

        for (const auto &neighborPos : neighbors)
        {
            size_t x = ROUND(neighborPos.x());
            size_t y = ROUND(neighborPos.y());

            if (x < 0 || y < 0 || x >= rows || y >= cols ||
                !isFree(x, y) || closedSet.count(hash(neighborPos)))
            {
                continue;
            }

            double tentative_g_cost = current.gCost + heuristic(current.position, neighborPos);
            double hCost = heuristic(neighborPos, this->destination);

            ThetaStarNode neighbor(neighborPos, tentative_g_cost, hCost, &current);

            if (current.parent != nullptr && lineOfSight(current.parent->position, neighbor.position))
            {
                if (tentative_g_cost < neighbor.gCost)
                {
                    neighbor.gCost = tentative_g_cost;
                    neighbor.fCost = tentative_g_cost + hCost;
                    neighbor.parent = current.parent;
                }
            }

            openSet.push(neighbor);
        }
    }
}

inline double EvasionThetaStar::heuristic(const Eigen::RowVector2d &p1, const Eigen::RowVector2d &p2)
{
    return (p1 - p2).norm();
}

bool EvasionThetaStar::lineOfSight(const Eigen::RowVector2d &p1, const Eigen::RowVector2d &p2)
{
    int x1 = p1.x(), y0 = p1.y();
    int x2 = p2.x(), y1 = p2.y();

    int dy = y1 - y0;
    int dx = x2 - x1;
    int f = 0;

    int sy = (dy > 0) ? 1 : -1;
    int sx = (dx > 0) ? 1 : -1;

    dy = std::abs(dy);
    dx = std::abs(dx);

    if (dx >= dy)
    {
        while (x1 != x2)
        {
            f += dy;
            if (f >= dx)
            {
                if (!isFree(y0 + ((sy - 1) / 2), x1 + ((sx - 1) / 2)))
                    return false;
                y0 += sy;
                f -= dx;
            }
            if (f != 0 && !isFree(y0 + ((sy - 1) / 2), x1 + ((sx - 1) / 2)))
                return false;
            if (dy == 0 && !isFree(y0, x1 + ((sx - 1) / 2)) && !isFree(y0 - 1, x1 + ((sx - 1) / 2)))
                return false;
            x1 += sx;
        }
    }
    else
    {
        while (y0 != y1)
        {
            f += dx;
            if (f >= dy)
            {
                if (!isFree(y0 + ((sy - 1) / 2), x1 + ((sx - 1) / 2)))
                    return false;
                x1 += sx;
                f -= dy;
            }
            if (f != 0 && !isFree(y0 + ((sy - 1) / 2), x1 + ((sx - 1) / 2)))
                return false;
            if (dx == 0 && !isFree(y0 + ((sy - 1) / 2), x1) && !isFree(y0 + ((sy - 1) / 2), x1 - 1))
                return false;
            y0 += sy;
        }
    }

    return true;
}
