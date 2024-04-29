#ifndef __SLAM_H__
#define __SLAM_H__

#include "Eigen/Dense"
#include "lidarSensor.h"

class ISlam
{
public:
    virtual ~ISlam(){};
    virtual void update(lidar_point_t *data) = 0;
    virtual const Eigen::MatrixXd *getGridMap() = 0;
    virtual const Eigen::RowVector2d getPosition() = 0;
    virtual const double getRotation() = 0;
};

#endif // __SLAM_H__
