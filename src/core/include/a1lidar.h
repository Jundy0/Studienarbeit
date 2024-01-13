#ifndef __A1LIDAR_H__
#define __A1LIDAR_H__

#include "rplidar.h"

typedef struct
{
    double radius;
    double angle;
    double x;
    double y;
    double quality;
    bool valid;
} point_t;

class IA1Lidar
{
public:
    virtual ~IA1Lidar(){};

    virtual void setPWM(int dutyCycle) = 0;
    virtual void startScan() = 0;
    virtual void stopScan() = 0;
    virtual void getScanData(point_t *data, size_t count) = 0;
};

class A1Lidar : public IA1Lidar
{
public:
    A1Lidar(std::string serialPort, int baudRate, int pwmPin);
    ~A1Lidar();

    void setPWM(int dutyCycle);
    void startScan();
    void stopScan();
    void getScanData(point_t *data, size_t count);

private:
    rp::standalone::rplidar::RPlidarDriver *drv;
    int pwmPin;
};

#endif // __A1LIDAR_H__
