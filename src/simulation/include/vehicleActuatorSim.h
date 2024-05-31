#ifndef __VEHICLE_ACTUATOR_SIM_H__
#define __VEHICLE_ACTUATOR_SIM_H__

#include <memory>

#include "vehicleActuator.h"
#include "vehicle.h"

class VehicleActuatorSim : public IVehicleActuator
{
public:
    VehicleActuatorSim(const std::shared_ptr<Vehicle> &vehicle);
    ~VehicleActuatorSim();

    void setForeward(double value);
    void setBackward(double value);
    void setLeft(double value);
    void setRight(double value);
    void update();

    const std::pair<Eigen::RowVector2d, double> getOdometry();

private:
    std::shared_ptr<Vehicle> vehicle;

    double foreward = .0;
    double backward = .0;
    double left = .0;
    double right = .0;

    sf::FloatRect lastPosition;
    float lastRotation;
};

#endif // __VEHICLE_ACTUATOR_SIM_H__
