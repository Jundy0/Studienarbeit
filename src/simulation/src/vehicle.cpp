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

sf::Sprite Vehicle::getSprite(bool collision)
{
    return this->vehicleSprite;
}

float Vehicle::getRotation()
{
    return this->rotation;
}

void Vehicle::moveForward()
{
    const float radiant = this->rotation * M_PI / 180.f;
    sf::Vector2f movement = sf::Vector2f(std::cos(radiant), std::sin(radiant)) * SPEED;
    this->position += movement;
}

void Vehicle::moveBack()
{
    const float radiant = this->rotation * M_PI / 180.f;
    sf::Vector2f movement = sf::Vector2f(std::cos(radiant), std::sin(radiant)) * SPEED;
    this->position -= movement;
}

void Vehicle::turnLeft()
{
    this->rotation -= SPEED;

    if (this->rotation < -360.f) // Reset Rotion to prevent overflow
    {
        this->rotation += 360.f;
    }
}

void Vehicle::turnRight()
{
    this->rotation += SPEED;

    if (this->rotation > 360.f) // Reset Rotion to prevent overflow
    {
        this->rotation -= 360.f;
    }
}

void Vehicle::update()
{
    this->vehicleSprite.setPosition(this->position);
    this->vehicleSprite.setRotation(this->rotation);
}
