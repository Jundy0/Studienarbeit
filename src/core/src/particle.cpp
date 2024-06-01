#include <cmath>
#include <chrono>

#include "particle.h"

Particle::Particle()
{
    OccupancyGrid occupancyGrid;
    PclHandler pclHandler;

    // Set starting position to the middle of the map
    position = {MAP_WIDTH / 2, MAP_HEIGHT / 2};
    // Zero equals east on the map
    rotationAngle = 0;
}

void Particle::update(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan)
{
    // Update grid map with data from previous scan and the position of the particle
    std::cout << "Updating grid map" << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();
    updateGridMap(firstScan);

    // Calculate position and rotation difference between the current scan and the previous scan and output computation time
    std::cout << "Updating position with pcl-registration\n"
              << std::endl;
    updatePosition(firstScan, secondScan);
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Update finished in " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms\n\n"
              << std::endl;
}

void Particle::update(Eigen::MatrixX2d currentScan, Eigen::RowVector2d positionDiff, double rotationDiff)
{
    // Update particle position and rotation using data read from the simulation
    std::cout << "Updating position with odometry data\n"
              << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();
    updatePositionWithOdometry(positionDiff, rotationDiff);

    // Update grid map with data from current scan and the position of the particle
    std::cout << "Updating grid map" << std::endl;
    updateGridMap(currentScan);
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Update finished in " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms\n\n"
              << std::endl;
}

//// DEBUG ONLY ////
void Particle::updateDebug(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan, Eigen::RowVector2d positionDiff, double rotationDiff)
{
    std::cout << "Updating grid map" << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();
    updateGridMap(firstScan);

    std::cout << "Updating position with pcl-registration\n"
              << std::endl;
    updatePositionDebug(firstScan, secondScan, positionDiff, rotationDiff);
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Update finished in " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms\n\n"
              << std::endl;
}
////////////////////

Eigen::MatrixXd *Particle::getGridMap()
{
    return occupancyGrid.getProbMap();
}

Eigen::RowVector2d Particle::getPosition()
{
    return position;
}

double Particle::getRotation()
{
    return rotationAngle;
}

void Particle::updatePosition(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan)
{
    // Use ICP to calculate transformation estimate between the two scans
    TransformationComponents transComp = pclHandler.computeTransformation(firstScan, secondScan, this->rotationAngle);
    std::cout << "Translation Vector: " << transComp.translation_vector << std::endl;
    std::cout << "Rotation: " << transComp.rotation_angle << std::endl;
    // Add calculated position and rotation diffence onto the particles position and rotation
    position += transComp.translation_vector;
    rotationAngle += transComp.rotation_angle;
}

void Particle::updatePositionWithOdometry(Eigen::RowVector2d positionDiff, double rotationDiff)
{
    // Add given position and rotation diffence onto the particles position and rotation
    position += positionDiff;
    rotationAngle += rotationDiff;
}

//// DEBUG ONLY ////
void Particle::updatePositionDebug(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan, Eigen::RowVector2d positionDiff, double rotationDiff)
{
    TransformationComponents transComp = pclHandler.computeTransformation(firstScan, secondScan, this->rotationAngle);
    std::cout << "Estimated Translation Vector: " << transComp.translation_vector << std::endl;
    std::cout << "Estimated Rotation: " << transComp.rotation_angle << std::endl;
    std::cout << "Real Translation Vector: " << positionDiff << ", Delta: " << transComp.translation_vector - positionDiff << std::endl;
    std::cout << "Real Rotation: " << rotationDiff << ", Delta: " << transComp.rotation_angle - rotationDiff << std::endl;
    position += transComp.translation_vector;
    rotationAngle += transComp.rotation_angle;
}
////////////////////

void Particle::updateGridMap(Eigen::MatrixX2d scan)
{
    occupancyGrid.updateProbMap(scan, position, rotationAngle);
}
