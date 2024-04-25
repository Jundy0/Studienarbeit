#ifndef __VEHICLE_ACTUATOR_SIM_H__
#define __VEHICLE_ACTUATOR_SIM_H__

#include "vehicleActuator.h"
#include "vehicle.h"

class VehicleActuatorSim : public IVehicleActuator
{
public:
    VehicleActuatorSim(Vehicle *vehicle);
    ~VehicleActuatorSim();

    void setForeward(double value);
    void setBackward(double value);
    void setLeft(double value);
    void setRight(double value);
    void update();

private:
    Vehicle *vehicle;

    double foreward = .0;
    double backward = .0;
    double left = .0;
    double right = .0;
};

#endif // __VEHICLE_ACTUATOR_SIM_H__
