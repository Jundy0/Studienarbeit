#include <cmath>
#include <chrono>

#include "particle.h"

Particle::Particle()
{
    OccupancyGrid occupancyGrid;
    IcpHandler icpHandler;

    position = {500, 500};
    rotationAngle = 0;
}

void Particle::update(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan)
{
    std::cout << "Updating Grid Map" << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();
    updateGridMap(firstScan);

    std::cout << "Updating Position\n"
              << std::endl;
    updatePosition(firstScan, secondScan);
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Update finished in " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms\n\n"
              << std::endl;
}

void Particle::visualizeGridMap()
{
    occupancyGrid.visualize();
}

Eigen::MatrixXd Particle::getGridMap()
{
    return occupancyGrid.getProbMap();
}

double Particle::getRotation()
{
    return rotationAngle;
}

Eigen::Vector2i Particle::getPosition()
{
    return position;
}

void Particle::updatePosition(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan)
{
    TransformationComponents transComp = icpHandler.call_icp(firstScan, secondScan);
    position -= transComp.translation_vector;
    rotationAngle -= transComp.rotation_angle;
}

void Particle::updateGridMap(Eigen::MatrixX2d scan)
{
    occupancyGrid.updateProbMap(scan, position, rotationAngle);
}
