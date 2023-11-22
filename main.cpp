#include <string>
#include <math.h>
#include "sl_lidar.h"

#define COUNT 8192
#define BAUDRATE 115200
#define SERIALPORT "/dev/ttyAMA0"

typedef struct
{
    double radius;
    double angle;
    double x;
    double y;
    double sinal_strength;
    bool valid;
} point_t;

int main()
{
    sl_result res;

    sl::ILidarDriver *drv = *sl::createLidarDriver();
    sl::IChannel *channel = *sl::createSerialPortChannel(SERIALPORT, BAUDRATE);

    res = drv->connect(channel);

    if (res == SL_RESULT_OK)
    {
        printf("Connected to Lidar\n");
    }
    else
    {
        printf("Failed to connect to Lidar\n");
        printf("Error code: %d\n", res);
        return -1;
    }

    res = drv->startScan(true, true);

    if (res == SL_RESULT_OK)
    {
        printf("Started scanning\n");
    }
    else
    {
        printf("Failed to start scanning\n");
        printf("Error code: %d\n", res);
        return -1;
    }

    size_t count = COUNT;

    sl_lidar_response_measurement_node_hq_t scanData[count];
    point_t points[count];

    res = drv->grabScanDataHq(scanData, count);

    if (res == SL_RESULT_OK)
    {
        printf("Grabbed scan data\n");
    }
    else
    {
        printf("Failed to grab scan data\n");
        printf("Error code: %d\n", res);
        return -1;
    }

    printf("Compute point values.\n");
    for (int i = 0; i < count; i++)
    {
        const double angle = scanData[i].angle_z_q14 * (90.f / 16384.f / (180.0f / M_PI));
        const double distance = scanData[i].dist_mm_q2 / 4000.0f;
        points[i].radius = distance;
        points[i].angle = angle;
        points[i].x = cos(angle) * distance;
        points[i].y = sin(angle) * distance;
        points[i].sinal_strength = scanData[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        points[i].valid = distance > 0;
    }

    printf("Scan data:\n");
    for (int i = 0; i < count; i++)
    {
        printf("Angle: %f\n", points[i].angle);
        printf("Distance: %f\n", points[i].radius);
        printf("x: %f\n", points[i].x);
        printf("y: %f\n", points[i].y);
        printf("Signal strength: %f\n", points[i].sinal_strength);
        printf("Valid: %d\n", points[i].valid);
    }

    delete drv;
    delete channel;

    return 0;
}
