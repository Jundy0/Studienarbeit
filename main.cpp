#include <string>
#include <math.h>
#include <cstring>

#include <pigpio.h>
#include "rplidar.h"
#include "a1lidar.h"

#define COUNT 8124
#define BAUDRATE 115200
#define SERIALPORT "/dev/ttyAMA0"
#define GPIO_PWM 18

int main()
{
    bool obsticleDetected = false;

    IA1Lidar *lidar = new A1Lidar(SERIALPORT, BAUDRATE, GPIO_PWM);

    lidar->startScan();

    point_t *points = (point_t *)malloc(sizeof(point_t) * COUNT);

    while (!obsticleDetected) 
    {
        lidar->getScanData(points, COUNT);

        for (int i = 0; i < COUNT; i++)
        {
            if (points[i].quality > 0 && points[i].radius < 2)
            {
                if (points[i].radius < 0.2)
                {
                    obsticleDetected = true;
                    printf("Obsticle detected at Angle: %f\n", points[i].angle);
                    break;
                }
            }
        }

        std::memset(points, 0, sizeof(point_t) * COUNT);
    }

    lidar->stopScan();

    free(points);

    delete lidar;
}
