#ifndef __A1LIDAR_H__
#define __A1LIDAR_H__

#include "lidar.h"

#include "rplidar.h"

class A1Lidar : public ILidar
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
