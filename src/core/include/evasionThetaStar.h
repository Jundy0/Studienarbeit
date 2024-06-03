#ifndef __EVASION_THETA_STAR_H__
#define __EVASION_THETA_STAR_H__

#include "evasionControl.h"

struct ThetaStarNode
{
    Eigen::RowVector2d position;
    double gCost;          // Cost from start to this node
    double hCost;          // Heuristic cost to goal
    double fCost;          // g_cost + h_cost
    ThetaStarNode *parent; // Parent node for path reconstruction

    ThetaStarNode(Eigen::RowVector2d pos, double g, double h, ThetaStarNode *p = nullptr)
        : position(pos), gCost(g), hCost(h), fCost(g + h), parent(p) {}

    bool operator>(const ThetaStarNode &other) const
    {
        return fCost > other.fCost;
    }
};

/// @brief The Class for Executing the ThetaStar Pathfinding Algorithm.
class EvasionThetaStar : public EvasionControl
{
public:
    /// @brief Create a new Evasion Control, that uses ThetaStar.
    /// @param vehicleActuator The Actuator to controll the Vehicle.
    EvasionThetaStar(const std::shared_ptr<IVehicleActuator> &vehicleActuator);

protected:
    /// @brief Execute the ThetaStar Pathfinding Algorithm.
    /// @note https://en.wikipedia.org/wiki/Theta*.
    void execute();

    /// @brief Heuristic Function to calculate the Euclidean Distance between two Points.
    /// @param p1 The first Point.
    /// @param p2 The second Point.
    /// @return The Euclidean Distance between the two Points.
    inline virtual double heuristic(const Eigen::RowVector2d &p1, const Eigen::RowVector2d &p2);

private:
    /// @brief Check if two Points have Line of Sight.
    /// @param p1 The first Point.
    /// @param p2 The second Point.
    /// @return True, if the two Points have Line of Sight.
    bool lineOfSight(const Eigen::RowVector2d &p1, const Eigen::RowVector2d &p2);
};

#endif // __EVASION_THETA_STAR_H__
