#ifndef __LIDAR_SIM_H__
#define __LIDAR_SIM_H__

#include "lidar.h"

class LidarSIM : public ILidar
{
public:
    LidarSIM();
    ~LidarSIM();

    void setPWM(int dutyCycle);
    void startScan();
    void stopScan();
    void getScanData(point_t *data, size_t count);
};

#endif // __LIDAR_SIM_H__
