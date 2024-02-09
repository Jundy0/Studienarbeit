#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#include <SFML/Graphics.hpp>

class Obstacle
{
public:
    Obstacle(float startX, float startY, float length, float height);
    ~Obstacle();

    sf::FloatRect getPosition();
    sf::RectangleShape getShape();

private:
    sf::Vector2f position;

    sf::RectangleShape obstacleShape;
};

#endif // __OBSTACLE_H__
