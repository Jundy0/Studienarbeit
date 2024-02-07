#ifndef __LIDAR_SENSOR_SIM_H__
#define __LIDAR_SENSOR_SIM_H__

#include "lidar.h"

class LidarSensorSim : public ILidarSensor
{
public:
    LidarSensorSim();
    ~LidarSensorSim();

    void setPWM(int dutyCycle);
    void startScan();
    void stopScan();
    void getScanData(lidar_point_t *data, size_t count);
};

#endif // __LIDAR_SENSOR_SIM_H__
