#include "vehicle.h"

#include <cmath>

Vehicle::Vehicle(float startX, float startY)
{
    this->position.x = startX;
    this->position.y = startY;

    this->vehicleTexture = sf::Texture();
    this->vehicleTexture.loadFromFile("../res/vehicle.png");

    this->vehicleSprite = sf::Sprite(this->vehicleTexture);
    this->vehicleSprite.setPosition(this->position);
    const sf::FloatRect pos = this->vehicleSprite.getGlobalBounds();
    this->vehicleSprite.setOrigin(pos.width / 2, pos.height / 2);
}

Vehicle::~Vehicle()
{
}

sf::FloatRect Vehicle::getPosition()
{
    return this->vehicleSprite.getGlobalBounds();
}

sf::Sprite Vehicle::getSprite()
{
    return this->vehicleSprite;
}

float Vehicle::getRotation()
{
    return this->vehicleRotation;
}

void Vehicle::setPosition(sf::Vector2f newPosition)
{
    this->position = newPosition;
}

void Vehicle::setRotation(float newRotation)
{
    this->vehicleRotation = newRotation;
}

void Vehicle::update()
{
    this->vehicleSprite.setPosition(this->position);
    this->vehicleSprite.setRotation(this->vehicleRotation);
}
