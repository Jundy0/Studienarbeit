#include "slamHandler.h"

SlamHandler::SlamHandler(size_t pointCount)
{
    this->pointCount = pointCount;
    this->currentScan = new Eigen::MatrixX2d(pointCount, 2);
    this->lastScan = new Eigen::MatrixX2d(pointCount, 2);
}

SlamHandler::~SlamHandler()
{
    delete this->currentScan;
    delete this->lastScan;
}

void SlamHandler::update(lidar_point_t *data, Eigen::RowVector2d positionDiff, double rotationDiff)
{
    for (size_t i = 0; i < this->pointCount; i++)
    {
        this->currentScan->row(i) = Eigen::RowVector2d{data[i].angle, data[i].radius};
    }

    if (this->initial)
    {
        this->initial = false;
    }
    else
    {
        this->particle.update(*this->lastScan, *this->currentScan);
    }

    std::swap(this->currentScan, this->lastScan);
}

const Eigen::MatrixXd *SlamHandler::getGridMap()
{
    return this->particle.getGridMap();
}

const Eigen::RowVector2d SlamHandler::getPosition()
{
    return this->particle.getPosition();
}

const double SlamHandler::getRotation()
{
    return this->particle.getRotation();
}
