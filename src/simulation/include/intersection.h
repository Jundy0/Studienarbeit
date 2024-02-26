#ifndef __INTERSECTION_H__
#define __INTERSECTION_H__

#include <SFML/Graphics.hpp>

#include <vector>

#include "obstacle.h"

/// @brief Gets the intersections of a Ray with the Obstacles and the Bounds of the Window.
/// @param rayOrigin The Origin of the Ray. 
/// @param rayAngle The Angle of the Ray in Radiant.
/// @param obstacles A List of Obstacles.
/// @param windowRect The Bounds of the Window as a Reactangle.
/// @param intersectionPoints A List of Points where the Ray intersected with an Obstacle or the Bounds of the Window. 
/// @return True, if an Intersection was found.
bool intersectsObstacles(const sf::Vector2f &rayOrigin, float rayAngle, const std::vector<Obstacle> &obstacles, sf::FloatRect &windowRect, std::vector<sf::Vector2f> &intersectionPoints);

/// @brief Gets the intersections of a Ray with a Reactangle.
/// @param rayOrigin The Origin of the Ray.
/// @param rayDirection The Direction of the Ray.
/// @param rect The Reactangle to check if the Ray intersects.
/// @param intersectionPoints A List of Points of Intersections, where the new Intersection should be appended.
/// @return True, if an Intersection was found.
bool intersectsRect(const sf::Vector2f &rayOrigin, const sf::Vector2f &rayDirection, const sf::FloatRect &rect, std::vector<sf::Vector2f> &intersectionPoints);

/// @brief Gets the intersections of a Ray with straight Line between two Points.
/// @param rayOrigin The Origin of the Ray.
/// @param rayDirection The Direction of the Ray.
/// @param p1 The start Point of the straight Line.
/// @param p2 The end Point of the straight Line.
/// @param intersectionPoints A List of Points of Intersections, where the new Intersection should be appended.
/// @return True, if an Intersection was found.
bool intersects(const sf::Vector2f &rayOrigin, const sf::Vector2f &rayDirection, const sf::Vector2f &p1, const sf::Vector2f &p2, std::vector<sf::Vector2f> &intersectionPoints);

/// @brief Calculate the squared Distance between two Points.
/// @remarks The squared Distance is used for Performance, since this Function is only used to compare two Distances.
/// @param p1 The first Point.
/// @param p2 The second Point.
/// @return The squared Distance between the two Points.
float distanceSquared(const sf::Vector2f& p1, const sf::Vector2f& p2);

#endif // __INTERSECTION_H__
