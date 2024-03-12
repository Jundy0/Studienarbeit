#include <string>
#include <math.h>
#include <vector>
#include <pigpio.h>
#include <matplot/matplot.h>
#include "rplidar.h"
#include "a1lidar.h"

#include <iostream>
#include <fstream>

#define COUNT 16000
#define BAUDRATE 115200
#define SERIALPORT "/dev/ttyAMA0"
#define GPIO_PWM 18

int main()
{
    auto f = matplot::figure(true);
    std::vector<double> theta;
    std::vector<double> rho;
    
    std::ofstream log;
    log.open("log.txt", std::ios_base::app);

    IA1Lidar *lidar = new A1Lidar(SERIALPORT, BAUDRATE, GPIO_PWM);

    lidar->startScan();

    point_t *points = (point_t *)malloc(sizeof(point_t) * COUNT);

    lidar->getScanData(points, COUNT);

    lidar->stopScan();

    for (int i = 0; i < COUNT; i++)
    {
        if (points[i].quality > 1)
  	{
	    log << points[i].angle << "," << points[i].radius << std::endl;
	    theta.push_back(points[i].angle);
            rho.push_back(points[i].radius);
        }
    }

    free(points);
    
    log.close();

    delete lidar;

    matplot::polarscatter(theta, rho, 1,"filled");
    f->save("./img/plot.jpg");
}
