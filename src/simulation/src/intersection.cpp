#include "intersection.h"

#include <cmath>

bool intersectsObstacles(const sf::Vector2f &rayOrigin, float rayAngle, std::vector<Obstacle> *obstacles, const sf::FloatRect &windowRect, std::vector<sf::Vector2f> &intersectionPoints)
{
    intersectionPoints.clear();

    // Round to prevent strange numbers after sin/cos
    float xDirection = std::round(std::cos(rayAngle) * 10000) / 10000;
    float yDirection = std::round(std::sin(rayAngle) * 10000) / 10000;

    sf::Vector2f rayDirection(xDirection, yDirection);

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
    bool intersects1 = intersects(rayOrigin, rayDirection, p1, p2, intersectionPoints);
    bool intersects2 = intersects(rayOrigin, rayDirection, p2, p3, intersectionPoints);
    bool intersects3 = intersects(rayOrigin, rayDirection, p4, p3, intersectionPoints);
    bool intersects4 = intersects(rayOrigin, rayDirection, p1, p4, intersectionPoints);
    return intersects1 || intersects2 || intersects3 || intersects4;
}

bool intersects(const sf::Vector2f &rayOrigin, const sf::Vector2f &rayDirection, const sf::Vector2f &p1, const sf::Vector2f &p2, std::vector<sf::Vector2f> &intersectionPoints)
{
    sf::Vector2f v1 = p1 - rayOrigin;
    sf::Vector2f v2 = p2 - rayOrigin;
    sf::Vector2f v12 = v2 - v1;

    const float cross = crossProduct(rayDirection, v12);

    if (cross == 0)
    {
        return false; // ray and edge are parallel
    }

    float t; // Parameter for Edge
    float s; // Parameter for Ray
    if (rayDirection.x != 0)
    {
        if (v12.x != 0)
        {
            t = (v1.x * rayDirection.y - v1.y * rayDirection.x) / (v12.x * rayDirection.y - v12.y * rayDirection.x);
        }
        else
        {
            t = (v1.x * rayDirection.y - v1.y * rayDirection.x) / (v12.y * rayDirection.x - v12.x * rayDirection.y);
        }
        s = (v1.x + t * v12.x) / rayDirection.x;
    }
    else
    {
        if (v12.x != 0)
        {
            t = (v1.y * rayDirection.x - v1.x * rayDirection.y) / (v12.x * rayDirection.y - v12.y * rayDirection.x);
        }
        else
        {
            t = (v1.y * rayDirection.x - v1.x * rayDirection.y) / (v12.y * rayDirection.x - v12.x * rayDirection.y);
        }
        s = (v1.y + t * v12.y) / rayDirection.y;
    }

    if (t >= 0 && t <= 1 && s >= 0) // Is between points and in positive direction of Ray
    {
        const sf::Vector2f intersectionPoint = v1 + rayOrigin + t * v12;
        intersectionPoints.push_back(intersectionPoint);

        return true;
    }

    return false;
}

inline float distanceSquared(const sf::Vector2f &p1, const sf::Vector2f &p2)
{
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return dx * dx + dy * dy;
}

inline float crossProduct(const sf::Vector2f &v1, const sf::Vector2f &v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}
