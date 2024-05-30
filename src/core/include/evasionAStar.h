#ifndef __EVASION_A_STAR_H__
#define __EVASION_A_STAR_H__

#include "evasionControl.h"

/// @brief The Class for Executing the AStar Pathfinding Algorithm.
class EvasionAStar : public EvasionControl
{
public:
    /// @brief Create a new Evasion Control, that uses AStar.
    /// @param vehicleActuator The Actuator to controll the Vehicle.
    EvasionAStar(IVehicleActuator *vehicleActuator);

protected:
    /// @brief Execute the AStar Pathfinding Algorithm.
    /// @note https://en.wikipedia.org/wiki/A*_search_algorithm.
    void execute();

private:
    /// @brief Heuristic function to calculate the Manhattan distance between the current Point and the destination Point.
    /// @param p The current Point.
    /// @return The Manhattan distance between the current Point and the destination Point.
    double heuristic(Eigen::RowVector2d p);
};

#endif // __EVASION_A_STAR_H__
