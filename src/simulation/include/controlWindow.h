#ifndef __CONTROL_WINDOW_H__
#define __CONTROL_WINDOW_H__

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/Audio.hpp>
#include <SFML/Network.hpp>

#include "lidarSensorSim.h"
#include "vehicleActuatorSim.h"
#include "selfdrivingVehicle.h"

#define WINDOW_WIDTH 500
#define WINDOW_HEIGHT 500

enum Mode
{
    placeObstacle = 1,
    placeDestination = 2,
};

class ControlWindow
{
public:
    ControlWindow(ILidarSensor *lidarSensor, IVehicleActuator *vehicleActuator, SelfdrivingVehicle *selfdrivingVehicle, Vehicle *vehicle, std::vector<Obstacle> *obstacles);
    ~ControlWindow();

    void update();
    void render();

    bool isOpen();
    void close();

private:
    sf::RenderWindow *window;
    sf::Event ev;
    sf::Clock clock;
    sf::Font font;
    sf::Text fpsDisplay;
    uint8_t fps;

    Mode mode = Mode::placeObstacle;

    Vehicle *vehicle;
    std::vector<Obstacle> *obstacles;
    bool placingObstacle = false;
    sf::Vector2f newObstacleP1;
    sf::Vector2f newObstacleP2;

    sf::Vector2f destination;

    ILidarSensor *lidarSensor;
    IVehicleActuator *vehicleActuator;
    SelfdrivingVehicle *selfdrivingVehicle;
    bool collision = false;

    void pollEvent();

    void saveScanAsCsv();
    uint16_t fileCount = 1;
};

#endif //__CONTROL_WINDOW_H__
