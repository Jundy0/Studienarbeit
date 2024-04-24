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

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800

class SimulationManager
{
public:
    SimulationManager();
    ~SimulationManager();

    void run();

private:
    sf::RenderWindow *window;
    sf::Event ev;

    Vehicle *vehicle;
    std::vector<Obstacle> obstacles;

    ILidarSensor *lidarSensor;
    lidar_point_t *data;
    bool collision = false;

    void update();
    void render();

    void pollEvent();

    void saveScanAsCsv();
    int fileCount = 1;
};

#endif // __SIMULATION_MANAGER_H__
