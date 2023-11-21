#include 'sdk/sl_lidar.h'

int baudrate = 115200;
std::string serialPort = "/dev/ttyAMA0";

// Create Driver and establish serial channel
ILidarDriver * drv = *createLidarDriver();
Result<IChannel*> channel = createSerialPortChannel(serialPort, baudrate);

// Connect Lidar via channel
auto res = (*lidar)->connect(*channel);




// Delete driver and channel
delete drv;
delete channel;

