#include "evasionThetaStar.h"

#include <queue>
#include <unordered_set>

#define ROUND(x) (size_t) std::round(x)

EvasionThetaStar::EvasionThetaStar(IVehicleActuator *vehicleActuator)
{
    this->vehicleActuator = vehicleActuator;
}

void EvasionThetaStar::execute()
{
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_set<long> closed_set;

    auto hash = [](const Eigen::RowVector2d &v)
    { return std::hash<double>()(v.x()) ^ std::hash<double>()(v.y()); };

    open_set.emplace(this->origin, 0.0, euclideanDistance(this->origin, this->destination));

    while (!open_set.empty())
    {
        Node current = open_set.top();
        open_set.pop();

        if (current.position == this->destination)
        {
            std::vector<Eigen::RowVector2d> path;
            Node *node = &current;
            while (node != nullptr)
            {
                path.push_back(node->position);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            this->path = path;
        }

        closed_set.insert(hash(current.position));

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

        for (const auto &neighbor_pos : neighbors)
        {
            if (neighbor_pos.x() < 0 || neighbor_pos.y() < 0 || neighbor_pos.x() >= (*this->map).rows() || neighbor_pos.y() >= (*this->map).cols() ||
                (*this->map)(ROUND(neighbor_pos.x()), ROUND(neighbor_pos.y())) == 1 || closed_set.count(hash(neighbor_pos)))
            {
                continue;
            }

            double tentative_g_cost = current.g_cost + euclideanDistance(current.position, neighbor_pos);
            double h_cost = euclideanDistance(neighbor_pos, this->destination);

            Node neighbor(neighbor_pos, tentative_g_cost, h_cost, &current);

            if (current.parent != nullptr && lineOfSight(current.parent->position, neighbor.position))
            {
                if (tentative_g_cost < neighbor.g_cost)
                {
                    neighbor.g_cost = tentative_g_cost;
                    neighbor.f_cost = tentative_g_cost + h_cost;
                    neighbor.parent = current.parent;
                }
            }

            open_set.push(neighbor);
        }
    }
}

double EvasionThetaStar::euclideanDistance(const Eigen::RowVector2d &a, const Eigen::RowVector2d &b)
{
    return (a - b).norm();
}

bool EvasionThetaStar::lineOfSight(const Eigen::RowVector2d &a, const Eigen::RowVector2d &b)
{
    int x0 = a.x(), y0 = a.y();
    int x1 = b.x(), y1 = b.y();

    int dy = y1 - y0;
    int dx = x1 - x0;
    int f = 0;

    int sy = (dy > 0) ? 1 : -1;
    int sx = (dx > 0) ? 1 : -1;

    dy = std::abs(dy);
    dx = std::abs(dx);

    if (dx >= dy)
    {
        while (x0 != x1)
        {
            f += dy;
            if (f >= dx)
            {
                if ((*this->map)(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)) == 1)
                    return false;
                y0 += sy;
                f -= dx;
            }
            if (f != 0 && (*this->map)(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)) == 1)
                return false;
            if (dy == 0 && (*this->map)(y0, x0 + ((sx - 1) / 2)) == 1 && (*this->map)(y0 - 1, x0 + ((sx - 1) / 2)) == 1)
                return false;
            x0 += sx;
        }
    }
    else
    {
        while (y0 != y1)
        {
            f += dx;
            if (f >= dy)
            {
                if ((*this->map)(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)) == 1)
                    return false;
                x0 += sx;
                f -= dy;
            }
            if (f != 0 && (*this->map)(y0 + ((sy - 1) / 2), x0 + ((sx - 1) / 2)) == 1)
                return false;
            if (dx == 0 && (*this->map)(y0 + ((sy - 1) / 2), x0) == 1 && (*this->map)(y0 + ((sy - 1) / 2), x0 - 1) == 1)
                return false;
            y0 += sy;
        }
    }

    return true;
}
