#ifndef __VEHICLE_H__
#define __VEHICLE_H__

#include <SFML/Graphics.hpp>

class Vehicle
{
public:
    Vehicle(float startX, float startY);
    ~Vehicle();

    sf::FloatRect getPosition();
    sf::Sprite getSprite();
    float getRotation();

    void setPosition(sf::Vector2f position);
    void setRotation(float newRotation);

    void update();

private:
    sf::Vector2f position;
    float vehicleRotation = 0.f;

    sf::Sprite vehicleSprite;
    sf::Texture vehicleTexture;
};

#endif // __VEHICLE_H__
