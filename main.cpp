#include <string>
#include <math.h>
#include <pigpio.h>
#include "rplidar.h"
#include "a1lidar.h"

#define COUNT 8192
#define BAUDRATE 115200
#define SERIALPORT "/dev/ttyAMA0"
#define GPIO_PWM 18

int main()
{
    IA1Lidar *lidar = new A1Lidar(SERIALPORT, BAUDRATE, GPIO_PWM);

    lidar->startScan();

    point_t *points = (point_t *)malloc(sizeof(point_t) * COUNT);

    lidar->getScanData(points, COUNT);

    lidar->stopScan();

    for (int i = 0; i < COUNT; i++)
    {
        printf("Angle: %f, Distance: %f, Valid: %s\n", points[i].x, points[i].y, points[i].valid ? "true" : "false");
    }

    free(points);

    delete lidar;
}
