#include "evasionControl.h"

EvasionControl::EvasionControl(IVehicleActuator *vehicleActuator)
{
    this->vehicleActuator = vehicleActuator;
}

EvasionControl::~EvasionControl()
{
}

void EvasionControl::setDestination(Eigen::RowVector2d destination)
{
    this->destination = destination;
}

void EvasionControl::update()
{
}
