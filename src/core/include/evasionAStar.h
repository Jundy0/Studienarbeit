#ifndef __EVASION_A_STAR_H__
#define __EVASION_A_STAR_H__

#include "evasionControl.h"

/// @brief A Node, used for the A Star Algorithm.
struct AStarNode
{
    Eigen::RowVector2d position; // Position of Node in Map.
    double gCost;                // Cost from Start to this Node.
    double hCost;                // Heuristic Cost.
    double fCost;                // Total Cost: gCost + hCost.
    AStarNode *parent;           // Parent Node for path reconstruction.

    /// @brief Create a new A Star Node.
    /// @param pos Position of Node in Map.
    /// @param g Cost from Start to this Node.
    /// @param h Heuristic Cost.
    /// @param p Parent Node for path reconstruction.
    AStarNode(Eigen::RowVector2d pos, double g, double h, AStarNode *p = nullptr)
        : position(pos), gCost(g), hCost(h), fCost(g + h), parent(p)
    {
    }

    /// @brief Compare two AStarNodes.
    /// @param other Another AStarNode.
    /// @return True, if the fCost of this AStarNode is greater than the other.
    bool operator>(const AStarNode &other) const
    {
        return fCost > other.fCost;
    }
};

/// @brief The Class for Executing the AStar Pathfinding Algorithm.
class EvasionAStar : public EvasionControl
{
public:
    /// @brief Create a new Evasion Control, that uses AStar.
    /// @param vehicleActuator The Actuator to controll the Vehicle.
    EvasionAStar(const std::shared_ptr<IVehicleActuator> &vehicleActuator);

protected:
    /// @brief Execute the AStar Pathfinding Algorithm.
    /// @note https://en.wikipedia.org/wiki/A*_search_algorithm.
    void execute();

    /// @brief Heuristic Function to calculate the Manhattan Distance between two Points.
    /// @param p1 The first Point.
    /// @param p2 The second Point.
    /// @return The Manhattan Distance between the two Points.
    inline virtual double heuristic(const Eigen::RowVector2d &p1, const Eigen::RowVector2d &p2);
};

#endif // __EVASION_A_STAR_H__
