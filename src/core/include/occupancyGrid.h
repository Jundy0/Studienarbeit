#ifndef __OCCUPANCY_GRID_H__
#define __OCCUPANCY_GRID_H__

#include "Eigen/Dense"
#include "settings.h"

/// @brief The Grid where the map for a particle is stored in.
class OccupancyGrid
{

public:
    /// @brief Constructor. Fills probMap with zeros.
    OccupancyGrid();

    /// @brief Updates the probability map based on scan data and the position and angle of the robot.
    /// @param scan The scan data. Points with angle and distance.
    /// @param robPos The position of the roboter at the time of when the scan data was acquiried.
    /// @param robRotAngle The rotation angle of the roboter at the time of when the scan data was acquired.
    void updateProbMap(const Eigen::MatrixX2d &scan, const Eigen::RowVector2d &robPos, double robRotAngle);

    /// @brief Returns the pointer of the current probability map.
    Eigen::MatrixXd *getProbMap();

private:
    Eigen::MatrixXd probMap; // The map where the probabilities for each pixel are stored at.

    /// @brief Returns X and Y coordinates of all points that are occupied and all points that are free based on given scan data.
    /// @param scan The scan data. Points with angle and distance.
    /// @param robPos The position of the roboter at the time of when the scan data was acquiried.
    /// @param robRotAngle The rotation angle of the roboter at the time of when the scan data was acquired.
    /// @return A pair of sets of occupied and free points.
    std::pair<Eigen::MatrixX2d, Eigen::MatrixX2d> getPoints(const Eigen::MatrixX2d &scan, const Eigen::RowVector2d &robPos, double robRotAngle);

    /// @brief Gets X and Y coordinates of all points between two given points.
    /// @param robPosX X coordinate of the roboter.
    /// @param robPosY Y coordinate of the robiter.
    /// @param x X coordinate of the target point.
    /// @param y Y coordinate of the target point.
    /// @return Matrix with x and y coordinates of all points between the roboter and the target point.
    Eigen::MatrixX2d bresenham(int robPosX, int robPosY, int x, int y);

    /// @brief Gets global x and y coordinates of a given point in polar coordinates (theta and radius).
    /// @param polarPoint A point in polar coordinates (theta and radius).
    /// @param robPos X and y coordinates of the roboter on the global map.
    /// @param robRotAngle Rotation angle of the roboter on the global map.
    /// @return Global x and y coordinate of the polar point.
    Eigen::RowVector2d polarToCartesian(const Eigen::RowVector2d &polarPoint, const Eigen::RowVector2d &robPos, double robRotAngle);
};

#endif // __OCCUPANCY_GRID_H__
