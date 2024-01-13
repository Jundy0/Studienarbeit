#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <cstddef>

typedef struct
{
    double radius;
    double angle;
    double x;
    double y;
    double quality;
    bool valid;
} point_t;

class ILidar
{
public:
    virtual ~ILidar(){};

    virtual void setPWM(int dutyCycle) = 0;
    virtual void startScan() = 0;
    virtual void stopScan() = 0;
    virtual void getScanData(point_t *data, size_t count) = 0;
};

#endif // __LIDAR_H__
