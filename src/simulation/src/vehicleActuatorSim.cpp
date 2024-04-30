#include "vehicleActuatorSim.h"

VehicleActuatorSim::VehicleActuatorSim(Vehicle *vehicle)
{
    this->vehicle = vehicle;
}

VehicleActuatorSim::~VehicleActuatorSim()
{
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

const std::pair<Eigen::RowVector2d, double> VehicleActuatorSim::getOdometry()
{
    sf::FloatRect currentPosition = this->vehicle->getPosition();
    float currentRotation = this->vehicle->getRotation();

    Eigen::RowVector2d positionDiff = {this->lastPosition.left - currentPosition.left, this->lastPosition.top - currentPosition.top};
    float rotationDiff = (this->lastRotation - currentRotation) * M_PI / 180; // Convert from DEG to RAD

    std::pair<Eigen::RowVector2d, double> odometry = std::make_pair(positionDiff, rotationDiff);

    this->lastPosition = currentPosition;
    this->lastRotation = currentRotation;

    return odometry;
}
