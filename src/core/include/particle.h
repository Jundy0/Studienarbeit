#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "Eigen/Dense"

#include "occupancyGrid.h"
#include "pclHandler.h"

/// @brief A representation of the roboter
class Particle
{
public:
    /// @brief Contructer. Creates instances of an OccupancyGrid and a PclHandler and initialises global position and rotation
    Particle();
    /// @brief Updates the particle using two consecutive sets of scan data
    /// @param firstScan First set of scan data composed of polar points
    /// @param secondScan Second set of scan data composed of polar points
    void update(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan);
    /// @brief Updates the particle using one set of scan data and the difference in position and rotation from the last scan made
    /// @param firstScan Set of scan data composed of polar points
    /// @param positionDiff Diffrence of the robots position since the last scan was made
    /// @param rotationDiff Diffrence of the robots rotation since the last scan was made
    void update(Eigen::MatrixX2d firstScan, Eigen::RowVector2d positionDiff, double rotationDiff);
    // Only for debug, uses ICP
    void updateDebug(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan, Eigen::RowVector2d positionDiff, double rotationDiff); 
    /// @brief Returns the pointer to the grid map of the occupancyGrid
    /// @return A pointer to the grid map of the occupancyGrid
    Eigen::MatrixXd *getGridMap();
    /// @brief Returns current x and y coordinates of the particle
    /// @return The current x and y coordinates of the particle
    Eigen::RowVector2d getPosition();
    /// @brief Returns current rotation of the particle
    /// @return The currentrotation of the particle
    double getRotation();

private:
    /// @brief An instance of the OccupancyGrid used for storing and interacting with the map of the particle
    OccupancyGrid occupancyGrid;
    /// @brief The interface for the pcl library used to do the icp computations
    PclHandler pclHandler;

    /// @brief The current position of the particle
    Eigen::RowVector2d position;
    /// @brief The current rotation angle of the particle
    double rotationAngle;

    /// @brief Updates the position and rotation of the particle using two consecutive sets of scan data
    /// @param firstScan First set of scan data composed of polar points
    /// @param secondScan Second set of scan data composed of polar points
    void updatePosition(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan);
    /// @brief Updates the position and rotation of the particle using one set of scan data and the difference in position and rotation from the last scan made
    /// @param firstScan Set of scan data composed of polar points
    /// @param positionDiff Diffrence of the robots position since the last scan was made
    /// @param rotationDiff Diffrence of the robots rotation since the last scan was made
    void updatePositionWithOdometry(Eigen::RowVector2d positionDiff, double rotationDiff);
    /// @brief Only for debug. Prints additional information
    void updatePositionDebug(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan, Eigen::RowVector2d positionDiff, double rotationDiff);
    /// @brief Updates the occupancyGrid probMap using a set of scan data
    /// @param scan Set of scan data composed of polar points
    void updateGridMap(Eigen::MatrixX2d scan);
};

#endif // __PARTICLE_H__
