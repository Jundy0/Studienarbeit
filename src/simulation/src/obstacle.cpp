#include "obstacle.h"

Obstacle::Obstacle(float startX, float startY, float length, float height)
{
    this->position.x = startX;
    this->position.y = startY;

    this->obstacleShape.setSize(sf::Vector2f(length, height));
    this->obstacleShape.setPosition(this->position);
    this->obstacleShape.setFillColor(sf::Color::Black);
}

Obstacle::~Obstacle()
{
}

sf::FloatRect Obstacle::getPosition()
{
    return this->obstacleShape.getGlobalBounds();
}

sf::RectangleShape Obstacle::getShape()
{
    return this->obstacleShape;
}
