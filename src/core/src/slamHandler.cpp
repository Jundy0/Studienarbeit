#include "slamHandler.h"

SlamHandler::SlamHandler(size_t pointCount)
    : pointCount(pointCount)
{
    this->currentScan = std::make_shared<Eigen::MatrixX2d>(pointCount, 2);
    this->lastScan = std::make_shared<Eigen::MatrixX2d>(pointCount, 2);
}

SlamHandler::~SlamHandler()
{
}

void SlamHandler::update(lidar_point_t *data, const Eigen::RowVector2d &positionDiff, double rotationDiff)
{
    // Set currentScan to the data of the scan
    for (size_t i = 0; i < this->pointCount; i++)
    {
        this->currentScan->row(i) = Eigen::RowVector2d{data[i].angle, data[i].radius};
    }

    // If update is called the first time no lastScan exists so update of the particle gets skipped
    if (this->initial)
    {
        this->initial = false;
    }
    else
    {
        if (this->useOdometry)
        {
            this->particle.update(this->currentScan, positionDiff, rotationDiff); // Only Ododmetry, no ICP
        }
        else
        {
            this->particle.update(this->lastScan, this->currentScan, positionDiff, rotationDiff);
        }
    }

    // currentScan gets moved to lastScan and gets overwritten next time update is called
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
