#ifndef __VISUALIZE_WINDOW_H__
#define __VISUALIZE_WINDOW_H__

#include <memory>

#include <SFML/Graphics.hpp>

#include "selfdrivingVehicle.h"
#include "simulationSettings.h"
#include "settings.h"

class VisualizeWindow
{
public:
    VisualizeWindow(const std::shared_ptr<SelfdrivingVehicle> &selfdrivingVehicle);
    ~VisualizeWindow();

    void update();
    void render();

    bool isOpen();
    void close();

private:
    std::unique_ptr<sf::RenderWindow> window;
    sf::Image visualizationImage;
    sf::Texture visualizationTexture;
    sf::Sprite visualizationSprite;
    sf::Sprite vehicleSprite;
    sf::Texture vehicleTexture;
    sf::Event ev;
    sf::Clock clock;
    sf::Font font;
    sf::Text fpsDisplay;
    uint8_t fps;

    std::shared_ptr<SelfdrivingVehicle> selfdrivingVehicle;

    void pollEvent();
};

#endif // __VISUALIZE_WINDOW_H__
