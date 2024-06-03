#include "particle.h"

Particle::Particle()
{
    // Set starting position to the middle of the map
    position = {MAP_WIDTH / 2, MAP_HEIGHT / 2};
    // Zero equals east on the map
    rotationAngle = 0;
}

void Particle::update(const std::shared_ptr<Eigen::MatrixX2d> &firstScan, const std::shared_ptr<Eigen::MatrixX2d> &secondScan, const Eigen::RowVector2d &positionDiff, double rotationDiff)
{
    // Update grid map with data from previous scan and the position of the particle
    std::cout << "Updating grid map" << std::endl;
    updateGridMap(*firstScan);

    // Calculate position and rotation difference between the current scan and the previous scan and output computation time
    std::cout << "Updating position with pcl-registration\n"
              << std::endl;

    TransformationComponents transComp = pclHandler.computeTransformation(*firstScan, *secondScan, this->rotationAngle);

    std::cout << "Estimated Translation Vector: " << transComp.translation_vector << std::endl;
    std::cout << "Estimated Rotation: " << transComp.rotation_angle << std::endl;
    std::cout << "Real Translation Vector: " << positionDiff << ", Delta: " << transComp.translation_vector - positionDiff << std::endl;
    std::cout << "Real Rotation: " << rotationDiff << ", Delta: " << transComp.rotation_angle - rotationDiff << std::endl;

    updatePosition(transComp.translation_vector, transComp.rotation_angle);
}

void Particle::update(const std::shared_ptr<Eigen::MatrixX2d> &currentScan, const Eigen::RowVector2d &positionDiff, double rotationDiff)
{
    // Update particle position and rotation using data read from the simulation
    std::cout << "Updating position with odometry data\n"
              << std::endl;
    updatePosition(positionDiff, rotationDiff);

    // Update grid map with data from current scan and the position of the particle
    std::cout << "Updating grid map" << std::endl;
    updateGridMap(*currentScan);
}

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

void Particle::updatePosition(const Eigen::RowVector2d &positionDiff, double rotationDiff)
{
    position += positionDiff;
    rotationAngle += rotationDiff;
}

void Particle::updateGridMap(const Eigen::MatrixX2d &scan)
{
    occupancyGrid.updateProbMap(scan, position, rotationAngle);
}
