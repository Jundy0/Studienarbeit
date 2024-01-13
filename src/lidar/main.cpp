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
    ILidar *lidar = new A1Lidar(SERIALPORT, BAUDRATE, GPIO_PWM);

    lidar->startScan();

    point_t *points = (point_t *)malloc(sizeof(point_t) * COUNT);

    lidar->getScanData(points, COUNT);

    lidar->stopScan();

    for (int i = 0; i < COUNT; i++)
    {
        if (points[i].quality > 1)
        {
            printf("Angle: %f, Radius: %f, Quality: %f\n", points[i].angle, points[i].radius, points[i].quality);
        }
    }

    free(points);

    delete lidar;
}
