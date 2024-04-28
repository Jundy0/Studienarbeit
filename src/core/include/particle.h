#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "Eigen/Dense"

#include "icpHandler.h"
#include "occupancyGrid.h"

class Particle
{
public:
    Particle();
    void update(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan);
    void visualizeGridMap();
    double getRotation();
    Eigen::Vector2i getPosition();
    Eigen::MatrixXd getGridMap();

private:
    OccupancyGrid occupancyGrid;
    IcpHandler icpHandler;

    Eigen::RowVector2i position;
    double rotationAngle;

    void updatePosition(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan);
    void updateGridMap(Eigen::MatrixX2d scan);
};

#endif // __PARTICLE_H__
