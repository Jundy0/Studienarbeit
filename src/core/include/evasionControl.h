#ifndef __EVASION_CONTROL_H__
#define __EVASION_CONTROL_H__

#include "Eigen/Dense"
#include "vehicleActuator.h"

#define INF 1e9

/// @brief The Class for Executing the Evasion Algorithm.
class EvasionControl
{
public:
    /// @brief Create a new Evasion Control.
    /// @param vehicleActuator The Actuator to controll the Vehicle.
    EvasionControl(IVehicleActuator *vehicleActuator);

    /// @brief Destroy the Evasion Control.
    ~EvasionControl();

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
    void update(const Eigen::MatrixXd *map, Eigen::RowVector2d position);

private:
    IVehicleActuator *vehicleActuator; // The Actuator to controll the Vehicle.

    const Eigen::MatrixXd *map;     // A Pointer to the current Map.
    Eigen::RowVector2d origin;      // The current origin Point.
    Eigen::RowVector2d destination; // The current destination Point.

    std::vector<Eigen::RowVector2d> path; // The current Path.

    /// @brief Calculate the Path using the AStar Algorithm.
    /// @note https://en.wikipedia.org/wiki/A*_search_algorithm
    void AStar();

    /// @brief Heuristic function to calculate the Manhattan distance between the current Point and the destination Point.
    /// @param p The current Point.
    /// @return The Manhattan distance between the current Point and the destination Point
    double heuristic(Eigen::RowVector2d p);
};

#endif // __EVASION_CONTROL_H__
