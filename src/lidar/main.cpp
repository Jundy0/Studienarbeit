#include <string>
#include <math.h>
#include <pigpio.h>
#include "rplidar.h"
#include "a1lidar.h"

#define COUNT 8124
#define BAUDRATE 115200
#define SERIALPORT "/dev/ttyAMA0"
#define GPIO_PWM 18

int main()
{
    ILidarSensor *lidarSensor = new A1LidarSensor(SERIALPORT, BAUDRATE, GPIO_PWM);

    lidarSensor->startScan();

    lidar_point_t *points = (lidar_point_t *)malloc(sizeof(lidar_point_t) * COUNT);

    lidarSensor->getScanData(points, COUNT);

    lidarSensor->stopScan();

    for (int i = 0; i < COUNT; i++)
    {
        if (points[i].quality > 1)
        {
            printf("Angle: %f, Radius: %f, Quality: %f\n", points[i].angle, points[i].radius, points[i].quality);
        }
    }

    free(points);

    delete lidarSensor;
}
