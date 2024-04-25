#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#include <SFML/Graphics.hpp>

class Obstacle
{
public:
    Obstacle(float startX, float startY, float length, float height);
    ~Obstacle();

    sf::FloatRect getPosition();
    sf::Sprite getSprite();

private:
    sf::Vector2f position;

    sf::Sprite obstacleSprite;

    static sf::Texture obstacleTexture;
    static size_t obstacleCount;
};

#endif // __OBSTACLE_H__
