#include "simulationManager.h"

SimulationManager::SimulationManager()
{
    this->vehicle = std::make_shared<Vehicle>(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);

    this->obstacles.emplace_back(100.f, 100.f, 50.f, 50.f);
    this->obstacles.emplace_back(190.f, 170.f, 100.f, 50.f);
    this->obstacles.emplace_back(500.f, 200.f, 100.f, 200.f);
    this->obstacles.emplace_back(300.f, 600.f, 300.f, 50.f);

    this->lidarSensor = std::make_shared<LidarSensorSim>(this->vehicle, this->obstacles);
    this->vehicleActuator = std::make_shared<VehicleActuatorSim>(this->vehicle);
    this->selfdrivingVehicle = std::make_shared<SelfdrivingVehicle>(this->lidarSensor, this->vehicleActuator);

    this->controlWindow = std::make_shared<ControlWindow>(this->lidarSensor, this->vehicleActuator, this->selfdrivingVehicle, this->vehicle, this->obstacles);
    this->controlWindow = std::make_shared<ControlWindow>(this->lidarSensor, this->vehicleActuator, this->selfdrivingVehicle, this->vehicle, this->obstacles);
    this->visualizeWindow = std::make_shared<VisualizeWindow>(this->selfdrivingVehicle);
}

SimulationManager::~SimulationManager()
{
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
