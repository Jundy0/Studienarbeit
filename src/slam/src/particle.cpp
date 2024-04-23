#include "../include/particle.h"



Particle::Particle() {
    occupancyGrid;
    icpHandler;

    position = {75, 75};
    rotationAngle = 0;
}

void Particle::update(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan) {
    updateGridMap(firstScan);
    updatePosition(firstScan, secondScan);
    occupancyGrid.visualize();
}

void Particle::updatePosition(Eigen::MatrixX2d firstScan, Eigen::MatrixX2d secondScan) {
    TransformationComponents transComp = icpHandler.call_icp(firstScan, secondScan);
    position = position + transComp.translation_vector;
    rotationAngle += transComp.rotation_angle;
}

void Particle::updateGridMap(Eigen::MatrixX2d scan) {
    occupancyGrid.updateProbMap(scan, position, rotationAngle);
}