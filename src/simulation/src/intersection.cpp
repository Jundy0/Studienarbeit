#include "intersection.h"

#include <cmath>

bool intersectsObstacles(const sf::Vector2f &rayOrigin, float rayAngle, std::vector<Obstacle> *obstacles, const sf::FloatRect &windowRect, std::vector<sf::Vector2f> &intersectionPoints)
{
    intersectionPoints.clear();

    sf::Vector2f rayDirection(std::cos(rayAngle), std::sin(rayAngle));

    for (auto &obstacle : *obstacles)
    {
        intersectsRect(rayOrigin, rayDirection, obstacle.getPosition(), intersectionPoints);
    }

    intersectsRect(rayOrigin, rayDirection, windowRect, intersectionPoints);

    std::sort(intersectionPoints.begin(), intersectionPoints.end(), [&](const sf::Vector2f &a, const sf::Vector2f &b)
              { return distanceSquared(rayOrigin, a) < distanceSquared(rayOrigin, b); });

    return !intersectionPoints.empty();
}

bool intersectsRect(const sf::Vector2f &rayOrigin, const sf::Vector2f &rayDirection, const sf::FloatRect &rect, std::vector<sf::Vector2f> &intersectionPoints)
{
    // Calculate intersection points with the rectangle's edges
    sf::Vector2f p1(rect.left, rect.top);
    sf::Vector2f p2(rect.left + rect.width, rect.top);
    sf::Vector2f p3(rect.left + rect.width, rect.top + rect.height);
    sf::Vector2f p4(rect.left, rect.top + rect.height);

    // Check for intersection with each edge
    if (intersects(rayOrigin, rayDirection, p1, p2, intersectionPoints) ||
        intersects(rayOrigin, rayDirection, p2, p3, intersectionPoints) ||
        intersects(rayOrigin, rayDirection, p3, p4, intersectionPoints) ||
        intersects(rayOrigin, rayDirection, p4, p1, intersectionPoints))
    {
        return true;
    }

    return false;
}

bool intersects(const sf::Vector2f &rayOrigin, const sf::Vector2f &rayDirection, const sf::Vector2f &p1, const sf::Vector2f &p2, std::vector<sf::Vector2f> &intersectionPoints)
{
    sf::Vector2f v1 = p1 - rayOrigin;
    sf::Vector2f v2 = p2 - rayOrigin;

    // Cross product to determine if the points are on the same side of the ray
    float cross = v1.x * rayDirection.y - v1.y * rayDirection.x;
    float cross2 = v2.x * rayDirection.y - v2.y * rayDirection.x;

    // If the signs are different, it means the points are on different sides of the ray
    if ((cross < 0 && cross2 > 0) || (cross > 0 && cross2 < 0))
    {
        // Calculate the intersection point using linear interpolation
        float t = cross / (cross - cross2);

        const sf::Vector2f intersectionPoint = sf::Vector2f(rayOrigin + t * rayDirection);
        intersectionPoints.push_back(intersectionPoint);

        return true;
    }

    return false;
}

float distanceSquared(const sf::Vector2f &p1, const sf::Vector2f &p2)
{
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return dx * dx + dy * dy;
}
