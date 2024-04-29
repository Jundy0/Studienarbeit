#ifndef __VISUALIZE_WINDOW_H__
#define __VISUALIZE_WINDOW_H__

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/Audio.hpp>
#include <SFML/Network.hpp>

#include "selfdrivingVehicle.h"

#include "simulationSettings.h"

class VisualizeWindow
{
public:
    VisualizeWindow(SelfdrivingVehicle *selfdrivingVehicle);
    ~VisualizeWindow();

    void update();
    void render();

    bool isOpen();
    void close();

private:
    sf::RenderWindow *window;
    sf::Image visualizationImage;
    sf::Texture visualizationTexture;
    sf::Sprite visualizationSprite;
    sf::Event ev;
    sf::Clock clock;
    sf::Font font;
    sf::Text fpsDisplay;
    uint8_t fps;

    SelfdrivingVehicle *selfdrivingVehicle;

    void pollEvent();
};

#endif // __VISUALIZE_WINDOW_H__
