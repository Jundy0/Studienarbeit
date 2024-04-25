#include <cmath>
#include <chrono>

#include "../include/particle.h"

Particle::Particle()
{
    OccupancyGrid occupancyGrid;
    IcpHandler icpHandler;

    position = {750, 750};
    rotationAngle = 0;
}

void Particle::update(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan)
{
    std::cout << "Updating Grid Map" << std::endl;
    updateGridMap(firstScan);

    std::cout << "Updating Position\n" << std::endl;
    auto t1 = chrono::high_resolution_clock::now();
    updatePosition(firstScan, secondScan);
    auto t2 = chrono::high_resolution_clock::now();
    std::cout << "Position update finished in " << chrono::duration_cast<chrono::milliseconds>(t2 - t1).count() << "ms\n\n" << std::endl;
}

void Particle::updatePosition(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan)
{
    TransformationComponents transComp = icpHandler.call_icp(firstScan, secondScan);
    position -= transComp.translation_vector; // Meter to Centimeter
    rotationAngle -= transComp.rotation_angle;
}

void Particle::updateGridMap(Eigen::MatrixX2d scan)
{
    occupancyGrid.updateProbMap(scan, position, rotationAngle);
}

void Particle::visualizeGridMap()
{
    occupancyGrid.visualize();
}