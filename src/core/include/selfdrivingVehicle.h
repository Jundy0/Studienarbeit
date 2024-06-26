#ifndef __SELFDRIVING_VEHICLE_H__
#define __SELFDRIVING_VEHICLE_H__

#include <memory>
#include <chrono>

#include "lidarSensor.h"
#include "vehicleActuator.h"
#include "slam.h"
#include "evasionControl.h"

#define SCAN_COUNT 720 // Count of Scans per one rotation.

/// @brief The Main Class for the Selfdriving Vehicle.
class SelfdrivingVehicle
{
public:
    /// @brief Create a new Selfdriving Vehicle.
    /// @param lidarSensor The Lidar Sensor to get the Scan Data from.
    /// @param vehicleActuator The Actuator to controll the Vehicle.
    SelfdrivingVehicle(const std::shared_ptr<ILidarSensor> &lidarSensor, const std::shared_ptr<IVehicleActuator> &vehicleActuator);

    /// @brief Destroy the Selfdriving Vehicle.
    ~SelfdrivingVehicle();

    /// @brief Get a readonly Pointer to the current Lidar Data.
    /// @return A readonly Pointer to the current Lidar Data.
    const lidar_point_t *getLidarDataPtr();

    /// @brief Get a readonly Pointer to the current Grid Map.
    /// @return A readonly Pointer to the current Grid Map.
    std::shared_ptr<Eigen::MatrixXd> getGridMap();

    /// @brief Get the current Position of the Vehicle.
    /// @return The current Position of the Vehicle.
    const Eigen::RowVector2d getPosition();

    /// @brief Get the current Rotation in RAD of the Vehicle.
    /// @return The current Rotation in RAD of the Vehicle.
    const double getRotation();

    /// @brief Get the current destination Point.
    /// @return The current destination Point.
    const Eigen::RowVector2d getDestination();

    /// @brief Get the current calculated Path.
    /// @return The current calculated Path.
    const std::vector<Eigen::RowVector2d> getPath();

    /// @brief Set a new destination Point.
    /// @param destination The new destination Point.
    void setDestination(Eigen::RowVector2d destination);

    /// @brief The Main update Function to call in the Main Loop. Process: Get Lidar Data -> Execute SLAM -> Execute Evasion -> Update Actuator.
    /// @note Should be called in the Main Loop.
    void update();

private:
    std::shared_ptr<ILidarSensor> lidarSensor;         // The Lidar Sensor to get the Scan Data from.
    std::shared_ptr<IVehicleActuator> vehicleActuator; // The Actuator to controll the Vehicle.
    std::unique_ptr<ISlam> slam;                       // The SLAM Algorithm to Map the Position of the Vehicle.
    lidar_point_t *lidarData;                          // The current Lidar Data.
    std::unique_ptr<EvasionControl> evasionControl;    // The Algorithm to evade obstacles.

    std::chrono::_V2::system_clock::time_point lastTime; // The last Timepoint.
};

#endif //__SELFDRIVING_VEHICLE_H__
