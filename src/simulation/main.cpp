#include <iostream>

#include "lidarSensorSim.h"
#include "vehicleActuatorSim.h"
#include "selfdrivingVehicle.h"
#include "simulationManager.h"

int main()
{
    SimulationManager simulationManager;
    simulationManager.run();
}
