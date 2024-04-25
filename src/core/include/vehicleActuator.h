#ifndef __VEHICLE_ACTUATOR_H__
#define __VEHICLE_ACTUATOR_H__

class IVehicleActuator
{
public:
    virtual ~IVehicleActuator(){};

    virtual void setForeward(double value) = 0;
    virtual void setBackward(double value) = 0;
    virtual void setLeft(double value) = 0;
    virtual void setRight(double value) = 0;
    virtual void update() = 0;
};

#endif // __VEHICLE_ACTUATOR_H__
