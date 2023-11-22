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

    point_t points1[COUNT];

    lidar->getScanData(points1, COUNT);

    lidar->stopScan();

    delete lidar;
}
