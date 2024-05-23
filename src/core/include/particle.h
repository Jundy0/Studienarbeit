#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "Eigen/Dense"

#include "occupancyGrid.h"
#include "pclHandler.h"

class Particle
{
public:
    Particle();
    void update(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan);
    void update(Eigen::MatrixX2d firstScan, Eigen::RowVector2d positionDiff, double rotationDiff);
    void updateDebug(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan, Eigen::RowVector2d positionDiff, double rotationDiff); // Only for debug
    void visualizeGridMap();
    Eigen::MatrixXd *getGridMap();
    Eigen::RowVector2d getPosition();
    double getRotation();

private:
    OccupancyGrid occupancyGrid;
    PclHandler pclHandler;

    Eigen::RowVector2d position;
    double rotationAngle;

    void updatePosition(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan);
    void updatePositionWithOdometry(Eigen::RowVector2d positionDiff, double rotationDiff);
    void updatePositionDebug(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan, Eigen::RowVector2d positionDiff, double rotationDiff);
    void updateGridMap(Eigen::MatrixX2d scan);
};

#endif // __PARTICLE_H__
