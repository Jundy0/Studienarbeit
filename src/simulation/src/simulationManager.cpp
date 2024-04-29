#include "simulationManager.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <cmath>

#define RADIUS 5

SimulationManager::SimulationManager()
{
    this->vehicle = new Vehicle(400.f, 400.f);

    Obstacle obstacle1 = Obstacle(100.f, 100.f, 50.f, 50.f);
    Obstacle obstacle2 = Obstacle(190.f, 170.f, 100.f, 50.f);
    Obstacle obstacle3 = Obstacle(500.f, 200.f, 100.f, 200.f);
    Obstacle obstacle4 = Obstacle(300.f, 600.f, 300.f, 50.f);

    this->obstacles.push_back(obstacle1);
    this->obstacles.push_back(obstacle2);
    this->obstacles.push_back(obstacle3);
    this->obstacles.push_back(obstacle4);

    this->lidarSensor = new LidarSensorSim(this->vehicle, this->obstacles);
    this->vehicleActuator = new VehicleActuatorSim(this->vehicle);
    this->selfdrivingVehicle = new SelfdrivingVehicle(this->lidarSensor, this->vehicleActuator);

    this->controlWindow = new ControlWindow(this->lidarSensor, this->vehicleActuator, this->selfdrivingVehicle, this->vehicle, &this->obstacles);
    this->visualizeWindow = new VisualizeWindow(this->selfdrivingVehicle);
}

SimulationManager::~SimulationManager()
{
    delete this->vehicle;
    delete this->lidarSensor;
    delete this->vehicleActuator;
    delete this->selfdrivingVehicle;
    delete this->controlWindow;
    delete this->visualizeWindow;
}

void SimulationManager::run()
{
    // Main Loop
    while (this->controlWindow->isOpen() && this->visualizeWindow->isOpen())
    {
        this->update();
        this->render();
    }

    // Close both windows if only one was closed
    if (this->controlWindow->isOpen())
    {
        this->controlWindow->close();
    }

    if (this->visualizeWindow->isOpen())
    {
        this->visualizeWindow->close();
    }
}

void SimulationManager::update()
{
    this->controlWindow->update();
    this->visualizeWindow->update();
}

void SimulationManager::render()
{
    this->controlWindow->render();
    this->visualizeWindow->render();
}
