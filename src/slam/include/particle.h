#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "../lib/eigen/Eigen/Dense"

#include "icp_handler.h"
#include "occupancy_grid.h"

class Particle
{

public:
    Particle();
    void update(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan);
    void visualizeGridMap();

private:
    OccupancyGrid occupancyGrid;
    IcpHandler icpHandler;

    Eigen::RowVector2i position;
    double rotationAngle;

    void updatePosition(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan);
    void updateGridMap(Eigen::MatrixX2d scan);
};

#endif // __PARTICLE_H__