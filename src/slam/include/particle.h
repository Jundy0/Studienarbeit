#include "../lib/eigen/Eigen/Dense"

#include "icp_handler.h"
#include "occupancy_grid.h"


class Particle
{

public:
    Particle();
    void update(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan);

private:
    OccupancyGrid occupancyGrid;
    IcpHandler icpHandler;

    Eigen::RowVector2i position;
    double rotationAngle;

    void updatePosition(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan);
    void updateGridMap(Eigen::MatrixX2d scan);
};