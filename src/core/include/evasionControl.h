#ifndef __EVASION_CONTROL_H__
#define __EVASION_CONTROL_H__

#include "Eigen/Dense"
#include "vehicleActuator.h"
#include "settings.h"

#define INF 1e9

/// @brief The Base Class for Executing an Evasion Algorithm.
class EvasionControl
{
public:
    /// @brief Destroy the Evasion Control.
    virtual ~EvasionControl(){};

    /// @brief Set a new destination Point.
    /// @param destination The new destination Point.
    void setDestination(Eigen::RowVector2d destination);

    /// @brief Get the current destination Point.
    /// @return The current destination Point.
    const Eigen::RowVector2d getDestination();

    /// @brief Get the current calculated Path.
    /// @return The current calculated Path.
    const std::vector<Eigen::RowVector2d> getPath();

    /// @brief The Main update Function to execute the Evasion Algorithm.
    /// @param map A Pointer to the current Map.
    /// @param position The current Position of the Vehicle.
    /// @param rotation The current Rotation of the Vehicle in radiant.
    void update(const Eigen::MatrixXd *map, Eigen::RowVector2d position, double rotation);

protected:
    IVehicleActuator *vehicleActuator; // The Actuator to controll the Vehicle.

    const Eigen::MatrixXd *map;     // A Pointer to the current Map.
    Eigen::RowVector2d origin;      // The current origin Point.
    Eigen::RowVector2d destination; // The current destination Point.
    double direction;               // The current direction in radiant.

    std::vector<Eigen::RowVector2d> path; // The current Path.

    /// @brief Execute the Pathfinding Algorithm.
    virtual void execute() = 0;
};

#endif // __EVASION_CONTROL_H__
