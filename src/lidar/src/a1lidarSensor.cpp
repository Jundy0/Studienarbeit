#include "a1lidarSensor.h"

#include <pigpio.h>
#include <math.h>
#include <unistd.h>

A1LidarSensor::A1LidarSensor(std::string serialPort, int baudRate, int pwmPin)
{
    this->pwmPin = pwmPin;
    gpioInitialise();
    gpioSetMode(this->pwmPin, PI_OUTPUT);

    drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

    u_result res = drv->connect(serialPort.c_str(), baudRate);

    if (res == RESULT_OK)
    {
        printf("Connected to Lidar\n");
    }
    else
    {
        printf("Failed to connect to Lidar\n");
        printf("Error code: %d\n", res);
        return;
    }
}

A1LidarSensor::~A1LidarSensor()
{
    delete drv;
}

void A1LidarSensor::setPWM(int dutyCycle)
{
    gpioHardwarePWM(this->pwmPin, 100, dutyCycle);
}

void A1LidarSensor::startScan()
{
    this->setPWM(750000);
    sleep(1);
    u_result res = drv->startScan(true, true);

    if (res == RESULT_OK)
    {
        printf("Started scanning\n");
    }
    else
    {
        printf("Failed to start scanning\n");
        printf("Error code: %d\n", res);
        return;
    }
}

void A1LidarSensor::stopScan()
{
    sleep(1);
    this->setPWM(0);
    drv->stop();
}

void A1LidarSensor::getScanData(lidar_point_t *data, size_t count)
{
    rplidar_response_measurement_node_hq_t scanData[count];
    u_result res = drv->grabScanDataHq(scanData, count);

    if (res == RESULT_OK)
    {
        printf("Grabbed scan data\n");
    }
    else
    {
        printf("Failed to grab scan data\n");
        printf("Error code: %d\n", res);
        return;
    }

    for (int i = 0; i < count; i++)
    {
        const double angle = scanData[i].angle_z_q14 * (90.f / 16384.f);
        const double distance = scanData[i].dist_mm_q2 / 4.0f;
        data[i].radius = distance;
        data[i].angle = angle * M_PI / 180;
        data[i].x = distance * cos(angle);
        data[i].y = distance * sin(angle);
        data[i].quality = scanData[i].quality;
        data[i].valid = scanData[i].quality > 7;
    }
}
