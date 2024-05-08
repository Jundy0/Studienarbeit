#ifndef __EVASION_CONTROL_H__
#define __EVASION_CONTROL_H__

#include "Eigen/Dense"
#include "vehicleActuator.h"

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

    /// @brief The Main update Function to execute the Evasion Algorithm.
    void update();

private:
    IVehicleActuator *vehicleActuator; // The Actuator to controll the Vehicle.

    Eigen::RowVector2d destination; // The current destination Point.
};

#endif // __EVASION_CONTROL_H__
