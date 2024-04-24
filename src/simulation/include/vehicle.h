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
    sf::RectangleShape getShape(bool collision = false);

    void moveLeft();
    void moveRight();
    void moveUp();
    void moveDown();

    void update();

private:
    sf::Vector2f position;

    sf::RectangleShape vehicleShape;
};

#endif // __VEHICLE_H__