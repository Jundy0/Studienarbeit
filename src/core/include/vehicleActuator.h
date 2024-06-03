#ifndef __VEHICLE_ACTUATOR_H__
#define __VEHICLE_ACTUATOR_H__

#include <Eigen/Dense>

/// @brief The Actuator to controll the Vehicle.
class IVehicleActuator
{
public:
    /// @brief Destroy the Vehicle Actuator.
    virtual ~IVehicleActuator(){};

    /// @brief Set the Value to drive Foreward.
    /// @param value The new Value.
    /// @note The value has to be between 0.0 and 1.0.
    virtual void setForeward(double value) = 0;

    /// @brief Set the Value to drive Backward.
    /// @param value The new Value.
    /// @note The value has to be between 0.0 and 1.0.
    virtual void setBackward(double value) = 0;

    /// @brief Set the Value to drive Left.
    /// @param value The new Value.
    /// @note The value has to be between 0.0 and 1.0.
    virtual void setLeft(double value) = 0;

    /// @brief Set the Value to drive Right.
    /// @param value The new Value.
    /// @note The value has to be between 0.0 and 1.0.
    virtual void setRight(double value) = 0;

    /// @brief The Main update Function to write the new Values to the Actuator/Motor of the Vehicle.
    /// @note Should be called in the Main Loop of the SelfdrivingVehicle.
    virtual void update() = 0;

    /// @brief Get the Odoemtry Data (Position and Rotation Difference) since the last call of the Method.
    /// @return A std::pair containing the Position and Rotation in RAD Difference, since the last Call.
    virtual const std::pair<Eigen::RowVector2d, double> getOdometry() = 0;
};

#endif // __VEHICLE_ACTUATOR_H__
