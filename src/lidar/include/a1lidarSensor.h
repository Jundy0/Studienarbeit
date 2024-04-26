#ifndef __A1LIDAR_H__
#define __A1LIDAR_H__

#include "lidarSensor.h"

#include "rplidar.h"

class A1LidarSensor : public ILidarSensor
{
public:
    A1LidarSensor(std::string serialPort, int baudRate, int pwmPin);
    ~A1LidarSensor();

    void setPWM(int dutyCycle);
    void startScan();
    void stopScan();
    void getScanData(lidar_point_t *data, size_t count);

private:
    rp::standalone::rplidar::RPlidarDriver *drv;
    int pwmPin;
};

#endif // __A1LIDAR_H__
