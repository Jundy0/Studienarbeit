#ifndef __EVASION_CONTROL_H__
#define __EVASION_CONTROL_H__

#include <memory>

#include "Eigen/Dense"
#include "vehicleActuator.h"
#include "settings.h"

#define ROUND(x) (size_t) std::round(x)

/// @brief The Base Class for Executing an Evasion Algorithm.
class EvasionControl
{
public:
    /// @brief Create a new Evasion Control.
    /// @param vehicleActuator The Actuator to controll the Vehicle.
    EvasionControl(const std::shared_ptr<IVehicleActuator> &vehicleActuator);

    /// @brief Destroy the Evasion Control.
    virtual ~EvasionControl(){};

    /// @brief Set a new destination Point.
    /// @param destination The new destination Point.
    void setDestination(const Eigen::RowVector2d &destination);

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
    void update(const Eigen::MatrixXd *map, const Eigen::RowVector2d &position, double rotation);

protected:
    std::shared_ptr<IVehicleActuator> vehicleActuator; // The Actuator to controll the Vehicle.

    const Eigen::MatrixXd *map;     // A Pointer to the current Map.
    Eigen::RowVector2d origin;      // The current origin Point.
    Eigen::RowVector2d destination; // The current destination Point.
    double direction;               // The current direction in radiant.

    std::vector<Eigen::RowVector2d> path; // The current Path.

    /// @brief Execute the Pathfinding Algorithm.
    virtual void execute() = 0;

    /// @brief Heuristic Function to calculate the Distance between two Points.
    /// @param p1 The first Point.
    /// @param p2 The second Point.
    /// @return The Distance between the two Points.
    inline virtual double heuristic(const Eigen::RowVector2d &p1, const Eigen::RowVector2d &p2) = 0;

    /// @brief Check, if the Field at the Coordiantes is free.
    /// @param x The x Coordinate.
    /// @param y The y Coordinate.
    /// @return True, if the Field is free.
    /// TODO: make inline????
    bool isFree(size_t x, size_t y);

private:
    /// @brief Print the Path in the console.
    void printPath();
};

#endif // __EVASION_CONTROL_H__
