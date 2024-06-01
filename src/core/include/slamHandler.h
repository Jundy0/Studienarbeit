#ifndef __SLAM_HANDLER_H__
#define __SLAM_HANDLER_H__

#include "slam.h"
#include "particle.h"

/// @brief Interface between the simulation and the particle used for slam
class SlamHandler : public ISlam
{
public:
    /// @brief Constructor. Sets pointCount and initializes currentScan and lastScan
    /// @param pointCount Number of points in each set of scan data
    SlamHandler(size_t pointCount);
    /// @brief Destructor. Deletes currentScan and lastScan
    ~SlamHandler();
    /// @brief Updates the particle
    /// @param data Current set of scan data
    /// @param positionDiff Difference of the vehicles position since the last update
    /// @param rotationDiff Difference of the vehicles rotation since the last update
    void update(lidar_point_t *data, Eigen::RowVector2d positionDiff, double rotationDiff);
    /// @brief Returns the pointer to the grid map of the occupancyGrid
    /// @return A pointer to the grid map of the occupancyGrid
    const Eigen::MatrixXd *getGridMap();
    /// @brief Returns current x and y coordinates of the particle
    /// @return The current x and y coordinates of the particle
    const Eigen::RowVector2d getPosition();
    /// @brief Returns current rotation of the particle
    /// @return The currentrotation of the particle
    const double getRotation();

private:
    /// @brief Flag that is true if no scan has been made.
    bool initial;
    /// @brief Number of points in each set of scan data
    size_t pointCount;

    /// @brief Instance of a particle used to represent the current state of the robot
    Particle particle;
    /// @brief Pointer to the current scan data
    Eigen::MatrixX2d *currentScan;
    /// @brief Pointer to the scan data acquired before the current scan data
    Eigen::MatrixX2d *lastScan;
};

#endif // __SLAM_HANDLER_H__
