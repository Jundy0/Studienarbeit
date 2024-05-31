#ifndef __LIDAR_SENSOR_SIM_H__
#define __LIDAR_SENSOR_SIM_H__

#include <vector>
#include <memory>

#include "lidarSensor.h"
#include "vehicle.h"
#include "obstacle.h"

class LidarSensorSim : public ILidarSensor
{
public:
    LidarSensorSim(const std::shared_ptr<Vehicle> &vehicle, std::vector<Obstacle> &obstacles);
    ~LidarSensorSim();

    void setPWM(int dutyCycle);
    void startScan();
    void stopScan();
    void getScanData(lidar_point_t *data, size_t count);

private:
    std::shared_ptr<Vehicle> vehicle;
    std::vector<Obstacle> *obstacles;
};

#endif // __LIDAR_SENSOR_SIM_H__
