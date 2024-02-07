#ifndef __LIDAR_SENSOR_H__
#define __LIDAR_SENSOR_H__

#include <cstddef>

typedef struct
{
    double radius;
    double angle;
    double x;
    double y;
    double quality;
    bool valid;
} lidar_point_t;

class ILidarSensor
{
public:
    virtual ~ILidarSensor(){};

    virtual void setPWM(int dutyCycle) = 0;
    virtual void startScan() = 0;
    virtual void stopScan() = 0;
    virtual void getScanData(lidar_point_t *data, size_t count) = 0;
};

#endif // __LIDAR_SENSOR_H__
