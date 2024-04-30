#ifndef __SLAM_HANDLER_H__
#define __SLAM_HANDLER_H__

#include "slam.h"
#include "particle.h"

class SlamHandler : public ISlam
{
public:
    SlamHandler(size_t pointCount);
    ~SlamHandler();
    void update(lidar_point_t *data, Eigen::RowVector2d positionDiff, double rotationDiff);
    const Eigen::MatrixXd *getGridMap();
    const Eigen::RowVector2d getPosition();
    const double getRotation();

private:
    bool initial;
    size_t pointCount;

    Particle particle;
    Eigen::MatrixX2d *currentScan;
    Eigen::MatrixX2d *lastScan;
};

#endif // __SLAM_HANDLER_H__
