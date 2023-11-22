#include <string>
#include "lib/rplidar_sdk/sdk/include/sl_lidar.h"

using namespace sl;

int main()
{
    sl_result res;

    int baudrate = 115200;
    std::string serialPort = "/dev/ttyAMA0";

    // Create Driver and establish serial channel
    ILidarDriver *drv;
    drv = *createLidarDriver();
    IChannel *channel = *createSerialPortChannel(serialPort, baudrate);

    // Connect Lidar via channel
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

    size_t count = 8192;

    sl_lidar_response_measurement_node_hq_t scanData[count];

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

    printf("Scan data:\n");
    for (int i = 0; i < count; i++)
    {
        printf("Angle: %f\n", scanData[i].angle_z_q14 * 90.f / (1 << 14));
        printf("Distance: %f\n", scanData[i].dist_mm_q2 / (1 << 2));
    }

    delete drv;
    delete channel;

    return 0;
}
