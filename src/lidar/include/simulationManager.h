#ifndef __SIMULATION_MANAGER_H__
#define __SIMULATION_MANAGER_H__

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/Audio.hpp>
#include <SFML/Network.hpp>

#include <Eigen/Dense>

#include <vector>

#include "vehicle.h"

#define WINDOW_WIDTH 1600
#define WINDOW_HEIGHT 1600

#define GRID_WIDTH 160
#define GRID_HEIGHT 160

class SimulationManager
{
public:
    SimulationManager();
    ~SimulationManager();

    void render(Eigen::MatrixXd gridMap, Eigen::Vector2i robPos, double robRotation);

private:
    sf::RenderWindow *window;
    sf::Image windowImage;
    sf::Texture windowTexture;
    sf::Sprite windowSprite;
    sf::Event ev;
    sf::Clock clock;
    sf::Font font;
    float lastFrame;

    Vehicle *vehicle;
};

#endif // __SIMULATION_MANAGER_H__
