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

enum Mode
{
    placeObstacle = 1,
    placeDestination = 2,
};

class SimulationManager
{
public:
    SimulationManager();
    ~SimulationManager();

    void run();

private:
    sf::RenderWindow *window;
    sf::Event ev;

    Mode mode = Mode::placeObstacle;

    Vehicle *vehicle;
    std::vector<Obstacle> obstacles;
    bool placingObstacle = false;
    sf::Vector2f newObstacleP1;
    sf::Vector2f newObstacleP2;

    sf::Vector2f destination;

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
