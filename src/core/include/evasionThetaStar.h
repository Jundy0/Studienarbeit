#ifndef __EVASION_THETA_STAR_H__
#define __EVASION_THETA_STAR_H__

#include "evasionControl.h"

struct Node
{
    Eigen::RowVector2d position;
    double g_cost; // Cost from start to this node
    double h_cost; // Heuristic cost to goal
    double f_cost; // g_cost + h_cost
    Node *parent;  // Parent node for path reconstruction

    Node(Eigen::RowVector2d pos, double g, double h, Node *p = nullptr)
        : position(pos), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}

    bool operator>(const Node &other) const
    {
        return f_cost > other.f_cost;
    }
};

/// @brief The Class for Executing the ThetaStar Pathfinding Algorithm.
class EvasionThetaStar : public EvasionControl
{
public:
    /// @brief Create a new Evasion Control, that uses ThetaStar.
    /// @param vehicleActuator The Actuator to controll the Vehicle.
    EvasionThetaStar(IVehicleActuator *vehicleActuator);

protected:
    /// @brief Execute the ThetaStar Pathfinding Algorithm.
    /// @note https://en.wikipedia.org/wiki/Theta*.
    void execute();

private:
    double euclideanDistance(const Eigen::RowVector2d &a, const Eigen::RowVector2d &b);
    bool lineOfSight(const Eigen::RowVector2d &a, const Eigen::RowVector2d &b);
};

#endif // __EVASION_THETA_STAR_H__
