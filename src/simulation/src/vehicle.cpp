#include "vehicle.h"

Vehicle::Vehicle() : Vehicle(5.f, 5.f)
{
}

Vehicle::Vehicle(float startX, float startY)
{
    this->position.x = startX;
    this->position.y = startY;

    this->vehicleShape.setSize(sf::Vector2f(50, 50));
    this->vehicleShape.setPosition(this->position);
    this->vehicleShape.setFillColor(sf::Color::Blue);
}

Vehicle::~Vehicle()
{
}

sf::FloatRect Vehicle::getPosition()
{
    return this->vehicleShape.getGlobalBounds();
}

sf::RectangleShape Vehicle::getShape()
{
    return this->vehicleShape;
}
void Vehicle::moveLeft()
{
    this->position.x -= SPEED;
}
void Vehicle::moveRight()
{
    this->position.x += SPEED;
}
void Vehicle::moveUp()
{
    this->position.y -= SPEED;
}
void Vehicle::moveDown()
{
    this->position.y += SPEED;
}

void Vehicle::update()
{
    this->vehicleShape.setPosition(position);
}
