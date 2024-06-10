#ifndef __EVASION_ADVANCED_A_STAR_H__
#define __EVASION_ADVANCED_A_STAR_H__

#include "evasionControl.h"

/// @brief A Node, used for the A Star Algorithm.
struct AdvancedAStarNode
{
    Eigen::RowVector2d position; // Position of Node in Map.
    double rotation;             // The Rotation of the Vehicle at the Node.
    double gCost;                // Cost from Start to this Node.
    double hCost;                // Heuristic Cost.
    double fCost;                // Total Cost: gCost + hCost.

    /// @brief Create a new AdvancedAStarNode.
    /// @param pos Position of Node in Map.
    /// @param g Cost from Start to this Node.
    /// @param h Heuristic Cost.
    /// @param p Parent Node for path reconstruction.
    AdvancedAStarNode(Eigen::RowVector2d pos, double rot, double g, double h)
        : position(pos), rotation(rot), gCost(g), hCost(h), fCost(g + h)
    {
    }

    /// @brief Create Empty AdvancedAStarNode.
    AdvancedAStarNode()
    {
    }

    /// @brief Compare two AStarNodes.
    /// @param other Another AdvancedAStarNode.
    /// @return True, if the fCost of this AdvancedAStarNode is greater than the other.
    bool operator>(const AdvancedAStarNode &other) const
    {
        return fCost > other.fCost;
    }
};

/// @brief The Class for Executing the AStar Pathfinding Algorithm using the Angle of the Vehicle.
class EvasionAdvancedAStar : public EvasionControl
{
public:
    /// @brief Create a new Evasion Control, that uses AStar.
    /// @param vehicleActuator The Actuator to controll the Vehicle.
    EvasionAdvancedAStar(const std::shared_ptr<IVehicleActuator> &vehicleActuator);

protected:
    /// @brief Execute the AStar Pathfinding Algorithm.
    /// @note https://en.wikipedia.org/wiki/A*_search_algorithm.
    /// @note https://gamedev.stackexchange.com/questions/150605/how-to-adapt-pathfinding-algorithms-to-restricted-movement
    void execute();

    /// @brief Heuristic Function to calculate the Manhattan Distance between two Points.
    /// @param p1 The first Point.
    /// @param p2 The second Point.
    /// @return The Manhattan Distance between the two Points.
    inline virtual double heuristic(const Eigen::RowVector2d &p1, const Eigen::RowVector2d &p2);
};

#endif // __EVASION_ADVANCED_A_STAR_H__
