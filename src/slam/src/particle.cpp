#include "../include/particle.h"



Particle::Particle() {
    OccupancyGrid occupancyGrid;
    IcpHandler icpHandler;

    Eigen::RowVector2i position = {50, 50};
    double rotationAngle = 0;
}

void Particle::update(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan) {
    updatePosition(firstScan, secondScan);
    updateGridMap(secondScan);
    occupancyGrid.visualize();
}

void Particle::updatePosition(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan) {
    TransformationComponents transComp = icpHandler.call_icp(firstScan, secondScan);
    position = {};
    rotationAngle = 0;
}

void Particle::updateGridMap(Eigen::MatrixX2d scan) {
    occupancyGrid.updateProbMap(scan, position, rotationAngle);
}