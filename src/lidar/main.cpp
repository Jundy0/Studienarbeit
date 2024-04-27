#include <string>
#include <math.h>
#include <signal.h>

#include "rplidar.h"
#include "a1lidarSensor.h"
#include "particle.h"
#include "Eigen/Dense"

#define COUNT 720
#define BAUDRATE 115200
#define SERIALPORT "/dev/ttyAMA0"
#define GPIO_PWM 18

Eigen::MatrixX2d pointsToMatrix(lidar_point_t *points);
void intHandler(int dummy);

static volatile int keepRunning = 1;

void intHandler(int dummy) 
{
    keepRunning = 0;
}

Eigen::MatrixX2d pointsToMatrix(lidar_point_t *points)
{
    Eigen::MatrixX2d scanMatrix(COUNT, 2);

    for (int i = 0; i < COUNT; i++)
    {
        scanMatrix.row(i) = Eigen::RowVector2d(points[i].angle, points[i].radius);
    }

    return scanMatrix;
}

int main()
{
    Particle particle;
    ILidarSensor *lidarSensor = new A1LidarSensor(SERIALPORT, BAUDRATE, GPIO_PWM);

    lidar_point_t *points = (lidar_point_t *)malloc(sizeof(lidar_point_t) * COUNT);
    
    Eigen::MatrixX2d currentScan(COUNT, 2);
    Eigen::MatrixX2d lastScan(COUNT, 2);

    signal(SIGINT, intHandler);

    int numberOfScans = 0;
    
    lidarSensor->startScan();

    lidarSensor->getScanData(points, COUNT);
    currentScan = pointsToMatrix(points);

    while (keepRunning)
    {
        lastScan = currentScan;
        lidarSensor->getScanData(points, COUNT);
        currentScan = pointsToMatrix(points);
        
        particle.update(lastScan, currentScan);
        
        numberOfScans++;

        if (numberOfScans % 5 == 0)
        {
            numberOfScans = 0;
            particle.visualizeGridMap();
        }
    }

    lidarSensor->stopScan();

    free(points);

    delete lidarSensor;
}