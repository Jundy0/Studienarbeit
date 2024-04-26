#ifndef __OCCUPANCY_GRID_H__
#define __OCCUPANCY_GRID_H__

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "Eigen/Dense"

class OccupancyGrid
{

public:
    OccupancyGrid();
    void updateProbMap(Eigen::MatrixX2d scan, Eigen::RowVector2i robPos, double robRotAngle);
    void visualize();

private:
    double probOcc;
    double probFree;
    int gridWidth;
    int gridHeight;
    int mapWidth;
    int mapHeight;

    Eigen::MatrixXd probMap;

    std::pair<Eigen::MatrixX2i, Eigen::MatrixX2i> getPoints(Eigen::MatrixX2d scan, Eigen::RowVector2i robPos, double robRotAngle);
    Eigen::MatrixX2i bresenham(int robPosX, int robPosY, int x, int y);
    Eigen::RowVector2i polarToCartesian(Eigen::RowVector2d polarPoint, Eigen::RowVector2i robPos, double robRotAngle);
};

#endif // __OCCUPANCY_GRID_H__
