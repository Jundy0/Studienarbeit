#include "obstacle.h"

Obstacle::Obstacle(float startX, float startY, float length, float height)
{
    this->position.x = startX;
    this->position.y = startY;

    if (this->obstacleCount == 0)
    {
        this->obstacleTexture = sf::Texture();
        this->obstacleTexture.loadFromFile("../res/obstacle.png");
        this->obstacleTexture.setRepeated(true);
    }

    this->obstacleSprite = sf::Sprite(this->obstacleTexture, sf::IntRect(0, 0, length, height));
    this->obstacleSprite.setPosition(this->position);

    this->obstacleCount++;
}

Obstacle::~Obstacle()
{
    this->obstacleCount--;
}

sf::FloatRect Obstacle::getPosition()
{
    return this->obstacleSprite.getGlobalBounds();
}

sf::Sprite Obstacle::getSprite()
{
    return this->obstacleSprite;
}

sf::Texture Obstacle::obstacleTexture;
size_t Obstacle::obstacleCount = 0;
