#include <string>
#include "lib/rplidar_sdk/sdk/include/sl_lidar.h"

using namespace sl;

int main()
{
    int baudrate = 115200;
    std::string serialPort = "/dev/ttyAMA0";

    // Create Driver and establish serial channel
    ILidarDriver *drv;
    drv = *createLidarDriver();
    IChannel *channel = *createSerialPortChannel(serialPort, baudrate);

    // Connect Lidar via channel
    auto res = drv->connect(channel);

    delete drv;
    delete channel;

    return 0;
}
