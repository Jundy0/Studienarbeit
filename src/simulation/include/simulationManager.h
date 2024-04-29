#ifndef __SIMULATION_MANAGER_H__
#define __SIMULATION_MANAGER_H__

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/Audio.hpp>
#include <SFML/Network.hpp>

#include <vector>

#include "vehicle.h"
#include "obstacle.h"
#include "lidarSensor.h"
#include "vehicleActuator.h"
#include "selfdrivingVehicle.h"
#include "controlWindow.h"
#include "visualizeWindow.h"

class SimulationManager
{
public:
    SimulationManager();
    ~SimulationManager();

    void run();

private:
    ControlWindow *controlWindow;
    VisualizeWindow *visualizeWindow;

    Vehicle *vehicle;
    std::vector<Obstacle> obstacles;

    ILidarSensor *lidarSensor;
    IVehicleActuator *vehicleActuator;
    SelfdrivingVehicle *selfdrivingVehicle;

    void update();
    void render();
};

#endif // __SIMULATION_MANAGER_H__
