#ifndef __SELFDRIVING_VEHICLE_H__
#define __SELFDRIVING_VEHICLE_H__

#include "lidarSensor.h"
#include "vehicleActuator.h"
#include "slam.h"

#define SCAN_COUNT 720 // Count of Scans per one rotation.

/// @brief The Main Class for the Selfdriving Vehicle.
class SelfdrivingVehicle
{
public:
    /// @brief Create a new Selfdriving Vehicle.
    /// @param lidarSensor The Lidar Sensor to get the Scan Data from.
    /// @param vehicleActuator The Actuator to controll the Vehicle.
    SelfdrivingVehicle(ILidarSensor *lidarSensor, IVehicleActuator *vehicleActuator);

    /// @brief Destroy the Selfdriving Vehicle.
    ~SelfdrivingVehicle();

    /// @brief Get a readonly Pointer to the current Lidar Data.
    /// @return A readonly Pointer to the current Lidar Data.
    const lidar_point_t *getLidarDataPtr();

    /// @brief Get a readonly Pointer to the current Grid Map.
    /// @return A readonly Pointer to the current Grid Map.
    const Eigen::MatrixXd *getGridMap();

    /// @brief Get the current Position of the Vehicle.
    /// @return The current Position of the Vehicle.
    const Eigen::RowVector2d getPosition();

    /// @brief Get the current Rotation in RAD of the Vehicle.
    /// @return The current Rotation in RAD of the Vehicle.
    const double getRotation();

    /// @brief The Main update Function to call in the Main Loop. Process: Get Lidar Data -> Execute SLAM -> Execute Evasion -> Update Actuator.
    /// @note Should be called in the Main Loop.
    void update();

private:
    ILidarSensor *lidarSensor;         // The Lidar Sensor to get the Scan Data from.
    IVehicleActuator *vehicleActuator; // The Actuator to controll the Vehicle.
    ISlam *slam;                       // The SLAM Algorithm to Map the Position of the Vehicle.
    lidar_point_t *lidarData;          // The current Lidar Data
};

#endif //__SELFDRIVING_VEHICLE_H__
