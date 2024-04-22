#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <../lib/eigen/Eigen/Dense>

#include "../include/occupancy_grid.h"

using namespace std;

OccupancyGrid::OccupancyGrid() {
    // Defining threshold
    probOcc = 0.5;
    probFree = -0.5;
    // Grid dimensions
    gridWidth = 100;
    gridHeight = 100;
    // Defining a matrix used to store probability values
    probMap = Eigen::MatrixXd::Zero(gridWidth, gridHeight);
};

void OccupancyGrid::updateProbMap(Eigen::MatrixX2d scan, Eigen::RowVector2i robPos, double robRotAngle) {
    std::pair<Eigen::MatrixX2i, Eigen::MatrixX2i> allPoints = getPoints(scan, robPos, robRotAngle);

    Eigen::MatrixX2i* occPoints = &allPoints.first;
    Eigen::MatrixX2i* freePoints = &allPoints.second;

    for (int i = 0; i < occPoints->rows(); i++) {
        int x = occPoints->coeff(i, 0);
        int y = occPoints->coeff(i, 1);

        if (probMap(x, y) < 1)
            probMap(x, y) += 0.1;
        
    }

    for (int i = 0; i < freePoints->rows(); i++) {
        int x = freePoints->coeff(i, 0);
        int y = freePoints->coeff(i, 1);

        if (probMap(x, y) > -1)
            probMap(x, y) -= 0.03;
    }

}

void OccupancyGrid::visualize() {
    const string DEFAULT_COLOR = "\033[0m";
    const string DARK_GREY_COLOR = "\033[48;5;234m";
    const string LIGHT_GREY_COLOR = "\033[48;5;248m";
    const string WHITE_COLOR = "\033[48;5;255m";

    cout << LIGHT_GREY_COLOR;

    for (int i = 0; i < gridWidth; i++) {
        for (int j = 0; j < gridHeight; j++) {
            if (probMap(i,j) >= probOcc)
                cout << DARK_GREY_COLOR << " " << LIGHT_GREY_COLOR;
            else if (probMap(i,j) <= probFree)
                cout << WHITE_COLOR << " " << LIGHT_GREY_COLOR;
            else
                cout << LIGHT_GREY_COLOR << " ";
        }
        cout << DEFAULT_COLOR << endl;
    }
    cout << DEFAULT_COLOR << endl;
}

std::pair<Eigen::MatrixX2i, Eigen::MatrixX2i> OccupancyGrid::getPoints(Eigen::MatrixX2d scan, Eigen::RowVector2i robPos, double robRotAngle) {
    
    Eigen::Matrix<int, -1, 2, Eigen::RowMajor> occPoints;
    Eigen::Matrix<int, -1, 2, Eigen::RowMajor> freePoints;
    
    Eigen::Matrix<double, -1, 2, Eigen::RowMajor> data = scan;

    
    
    for (int i = 0; i < data.rows(); i++) {
        Eigen::RowVector2d polarPoint = data.block<1,2>(i,0);
        Eigen::RowVector2i cartPoint = polarToCartesian(polarPoint, robPos, robRotAngle);
        occPoints.conservativeResize(occPoints.rows()+1,Eigen::NoChange);
        occPoints.row(occPoints.rows()-1) = cartPoint;
        
        Eigen::MatrixX2i bresenhamPoints = bresenham(robPos[0], robPos[1], cartPoint[0], cartPoint[1]);
        for (int j = 0; j < bresenhamPoints.rows(); j++) {
            freePoints.conservativeResize(freePoints.rows()+1,Eigen::NoChange);
            freePoints.row(freePoints.rows()-1) = bresenhamPoints.block<1,2>(j,0);
        }
    }

    return std::make_pair(occPoints, freePoints);
}

Eigen::MatrixX2i OccupancyGrid::bresenham(int robPosX, int robPosY, int x, int y) {
    Eigen::Matrix<int, -1, 2, Eigen::RowMajor> points;
    int x1 = robPosX, y1 = robPosY;
    int x2 = x, y2 = y;
    
    // Move endpoint towards robot to exclude lidar point
    if (x1 > x2)
        x2++;
    else
        x2--;

    if (y1 > y2)
        y2++;
    else
        y2--;
    

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x1 == x2 && y1 == y2) 
            break;
        
        int e2 = 2 * err;
        
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }

        points.conservativeResize(points.rows()+1,Eigen::NoChange);
        points.row(points.rows()-1) = Eigen::RowVector2i{x1, y1};
    }
    return points;
}

Eigen::RowVector2i OccupancyGrid::polarToCartesian(Eigen::RowVector2d polarPoint, Eigen::RowVector2i robPos, double robRotAngle) {
    Eigen::RowVector2i cartPoint;
    
    double theta = polarPoint[0] + robRotAngle;
    double r = polarPoint[1] * 10 * 100; // Meter to Centimeter plus fitst digit after comma  

    cartPoint[0] = round(r * cos(theta) / 10) + robPos[0];
    cartPoint[1] = round(r * sin(theta) / 10) + robPos[1];

    return cartPoint;
};
