#include "vehicleActuatorSim.h"

#include <iostream>

VehicleActuatorSim::VehicleActuatorSim(Vehicle *vehicle)
{
    this->vehicle = vehicle;
    std::cout << "VehicleActuatorSim: Constructor" << std::endl;
}

VehicleActuatorSim::~VehicleActuatorSim()
{
    std::cout << "VehicleActuatorSim: Destructor" << std::endl;
}

void VehicleActuatorSim::setForeward(double value)
{
    if (value <= .0)
    {
        this->foreward = .0;
    }
    else if (value >= 1.0)
    {
        this->foreward = 1.0;
    }
    else
    {
        this->foreward = value;
    }
}

void VehicleActuatorSim::setBackward(double value)
{
    if (value <= .0)
    {
        this->backward = .0;
    }
    else if (value >= 1.0)
    {
        this->backward = 1.0;
    }
    else
    {
        this->backward = value;
    }
}

void VehicleActuatorSim::setLeft(double value)
{
    if (value <= .0)
    {
        this->left = .0;
    }
    else if (value >= 1.0)
    {
        this->left = 1.0;
    }
    else
    {
        this->left = value;
    }
}

void VehicleActuatorSim::setRight(double value)
{
    if (value <= .0)
    {
        this->right = .0;
    }
    else if (value >= 1.0)
    {
        this->right = 1.0;
    }
    else
    {
        this->right = value;
    }
}

void VehicleActuatorSim::update()
{
    if (this->foreward > .0 && this->backward == .0)
    {
        if (this->left > .0 && this->right == .0)
        {
            this->vehicle->turnLeft();
        }
        else if (this->right > .0 && this->left == .0)
        {
            this->vehicle->turnRight();
        }
        this->vehicle->moveForward();
    }
    else if (this->backward > .0 && this->foreward == .0)
    {
        if (this->left > .0 && this->right == .0)
        {
            this->vehicle->turnRight(); // Maybe change to Left but needs to be Right cause of backwards
        }
        else if (this->right > .0 && this->left == .0)
        {
            this->vehicle->turnLeft(); // Maybe change to Right but needs to be Left cause of backwards
        }
        this->vehicle->moveBack();
    }

    this->vehicle->update();
}
