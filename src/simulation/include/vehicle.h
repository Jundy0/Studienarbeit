#ifndef __VEHICLE_H__
#define __VEHICLE_H__

#include <SFML/Graphics.hpp>

#define SPEED 1.f

class Vehicle
{
public:
    Vehicle(float startX, float startY);
    ~Vehicle();

    sf::FloatRect getPosition();
    sf::Sprite getSprite(bool collision = false);

    void moveForward();
    void moveBack();
    void turnLeft();
    void turnRight();

    void update();

private:
    sf::Vector2f position;
    float rotation = 0.f;

    sf::Sprite vehicleSprite;
    sf::Texture vehicleTexture;
};

#endif // __VEHICLE_H__
