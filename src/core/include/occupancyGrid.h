#ifndef __OCCUPANCY_GRID_H__
#define __OCCUPANCY_GRID_H__

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "Eigen/Dense"

#define MAP_WIDTH 5000
#define MAP_HEIGHT 5000

#define GRID_WIDTH 500
#define GRID_HEIGHT 500

#define PROB_OCC 0.4
#define PROB_FREE -0.2

class OccupancyGrid
{

public:
    OccupancyGrid();
    void updateProbMap(Eigen::MatrixX2d scan, Eigen::RowVector2d robPos, double robRotAngle);
    void visualize();
    Eigen::MatrixXd *getProbMap();

private:
    Eigen::MatrixXd probMap;

    std::pair<Eigen::MatrixX2d, Eigen::MatrixX2d> getPoints(Eigen::MatrixX2d scan, Eigen::RowVector2d robPos, double robRotAngle);
    Eigen::MatrixX2d bresenham(int robPosX, int robPosY, int x, int y);
    Eigen::RowVector2d polarToCartesian(Eigen::RowVector2d polarPoint, Eigen::RowVector2d robPos, double robRotAngle);
};

#endif // __OCCUPANCY_GRID_H__
