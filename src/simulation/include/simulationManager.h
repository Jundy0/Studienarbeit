#ifndef __SIMULATION_MANAGER_H__
#define __SIMULATION_MANAGER_H__

#include <vector>
#include <memory>

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
    std::shared_ptr<ControlWindow> controlWindow;
    std::shared_ptr<VisualizeWindow> visualizeWindow;

    std::shared_ptr<Vehicle> vehicle;
    std::vector<Obstacle> obstacles;

    std::shared_ptr<ILidarSensor> lidarSensor;
    std::shared_ptr<IVehicleActuator> vehicleActuator;
    std::shared_ptr<SelfdrivingVehicle> selfdrivingVehicle;

    void update();
    void render();
};

#endif // __SIMULATION_MANAGER_H__
