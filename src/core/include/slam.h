#ifndef __SLAM_H__
#define __SLAM_H__

#include <Eigen/Dense>

#include "lidarSensor.h"

/// @brief The Class for Executing the SLAM Algorithm.
class ISlam
{
public:
    /// @brief Destroy the SLAM Object.
    virtual ~ISlam(){};

    /// @brief Get the current calculated Grid Map.
    /// @return The current calculated Grid Map.
    virtual const Eigen::MatrixXd *getGridMap() = 0;

    /// @brief Get the current Calculated Position of the Vehicle.
    /// @return The current Calculated Position of the Vehicle.
    virtual const Eigen::RowVector2d getPosition() = 0;

    /// @brief Get the current calculated Rotation in RAD of the Vehicle.
    /// @return The current calculated Rotation in RAD of the Vehicle.
    virtual const double getRotation() = 0;

    /// @brief The Main update Function to execute the SLAM Algorithm.
    /// @param data The new Scan Data from the Lidar.
    /// @param positionDiff The Position Difference since the last call of the Method.
    /// @param rotationDiff The Rotation Difference in RAD since the last call of the Method.
    /// @note Not every SLAM Algorithm actually uses the Odometry Data (Position and Rotation Difference).
    virtual void update(lidar_point_t *data, const Eigen::RowVector2d &positionDiff, double rotationDiff) = 0;
};

#endif // __SLAM_H__
