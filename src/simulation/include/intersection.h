#ifndef __INTERSECTION_H__
#define __INTERSECTION_H__

#include <SFML/Graphics.hpp>

#include <vector>

#include "obstacle.h"

bool intersectsObstacles(const sf::Vector2f& rayOrigin, float rayAngle, const std::vector<Obstacle>& obstacles, sf::FloatRect& windowRect, sf::Vector2f& intersectionPoint);
bool intersectsRect(const sf::Vector2f& rayOrigin, const sf::Vector2f& rayDirection, const sf::FloatRect& rect, , sf::Vector2f& intersectionPoint);
bool intersects(const sf::Vector2f& rayOrigin, const sf::Vector2f& rayDirection, const sf::Vector2f& p1, const sf::Vector2f& p2, sf::Vector2f& intersectionPoint);

#endif // __INTERSECTION_H__
