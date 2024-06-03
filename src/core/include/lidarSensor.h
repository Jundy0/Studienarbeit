#ifndef __LIDAR_SENSOR_H__
#define __LIDAR_SENSOR_H__

/// @brief A Point that is scanned by the Lidar.
typedef struct
{
    /// @brief The Radius/Distance from the Lidar/Vehicle to the Point.
    double radius;
    /// @brief The Angle of the Point in Radiant.
    double angle;
    /// @brief The x Coordinate of the Point on the Grid.
    /// @note This Value is empty by default and must be calculated.
    double x;
    /// @brief The y Coordinate of the Point on the Grid.
    /// @note This Value is empty by default and must be calculated.
    double y;
    /// @brief The Quality of the Point.
    double quality;
    /// @brief Indicator, if the Point is valid.
    bool valid;
} lidar_point_t;

/// @brief The Lidar Sensor to get the Scan Data from.
class ILidarSensor
{
public:
    /// @brief Destroy the Lidar Sensor.
    virtual ~ILidarSensor(){};

    /// @brief Set the PWM Value for the Motor of the Lidar.
    /// @param dutyCycle The Duty Cycle of the PWM Signal.
    virtual void setPWM(int dutyCycle) = 0;

    /// @brief Start the Scan of the Lidar Sensor.
    virtual void startScan() = 0;

    /// @brief Stop the Scan of the Lidar Sensor.
    virtual void stopScan() = 0;

    /// @brief Get the Scan Data of the Lidar Sensor.
    /// @param data The Pointer in which the Data is saved.
    /// @param count The Count of Points to save in the Pointer.
    virtual void getScanData(lidar_point_t *data, size_t count) = 0;
};

#endif // __LIDAR_SENSOR_H__
