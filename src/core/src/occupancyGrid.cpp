#include "occupancyGrid.h"

OccupancyGrid::OccupancyGrid()
{
    // Defines the matrix used to store probability values and fills it with zeroes
    probMap = std::make_unique<Eigen::MatrixXd>(GRID_HEIGHT, GRID_WIDTH);
    probMapCpy = std::make_shared<Eigen::MatrixXd>(GRID_HEIGHT, GRID_WIDTH);
    *probMap = Eigen::MatrixXd::Zero(GRID_HEIGHT, GRID_WIDTH);
}

OccupancyGrid::~OccupancyGrid()
{
}

void OccupancyGrid::updateProbMap(const Eigen::MatrixX2d &scan, const Eigen::RowVector2d &robPos, double robRotAngle)
{
    // Gets coordinates of free and occupied points and saves them into matrix
    std::pair<Eigen::MatrixX2d, Eigen::MatrixX2d> allPoints = getPoints(scan, robPos, robRotAngle);

    Eigen::MatrixX2d *occPoints = &allPoints.first;
    Eigen::MatrixX2d *freePoints = &allPoints.second;

    // Updates values in the probMap for occupied points
    for (int i = 0; i < occPoints->rows(); i++)
    {
        // Gets x and y for each point from the matrix
        // Must be divided by the ratio between map and grid since the grind is a fraction of the size to allow for some error
        // (e.g. 1x1 in the grid is 10x10 in the map so all the points on the map that lay in this 10x10 area will change the value of probability at that single point in the grid)
        int x = occPoints->coeff(i, 0) / (MAP_WIDTH / GRID_WIDTH);
        int y = occPoints->coeff(i, 1) / (MAP_WIDTH / GRID_WIDTH);

        if ((*probMap)(y, x) < PROB_OCC_THRES)
            (*probMap)(y, x) += DELTA_OCC;
    }

    // Updates values in the probMap for free points
    for (int i = 0; i < freePoints->rows(); i++)
    {
        // Gets x and y for each point from the matrix
        // Must be divided by the ratio between map and grid since the grind is a fraction of the size to allow for some error
        int x = freePoints->coeff(i, 0) / (MAP_WIDTH / GRID_WIDTH);
        int y = freePoints->coeff(i, 1) / (MAP_WIDTH / GRID_WIDTH);

        if ((*probMap)(y, x) > PROB_FREE_THRES)
            (*probMap)(y, x) += DELTA_FREE;
    }

    // Copy Map
    *this->probMapCpy = *this->probMap;
}

std::shared_ptr<Eigen::MatrixXd> OccupancyGrid::getProbMap()
{
    return this->probMapCpy;
}

std::pair<Eigen::MatrixX2d, Eigen::MatrixX2d> OccupancyGrid::getPoints(const Eigen::MatrixX2d &scan, const Eigen::RowVector2d &robPos, double robRotAngle)
{
    // Define X,2 Matrices for occupied and free points.
    // RowMajor stores the matrix row wise in memory and is used since each point is represented by one row in the matrix.
    Eigen::Matrix<double, -1, 2, Eigen::RowMajor> occPoints;
    Eigen::Matrix<double, -1, 2, Eigen::RowMajor> freePoints;

    Eigen::Matrix<double, -1, 2, Eigen::RowMajor> data = scan;

    for (int i = 0; i < data.rows(); i++)
    {
        // Converts each of the polar points from the scan data into cartesian and stores it in occPoints
        Eigen::RowVector2d polarPoint = data.block<1, 2>(i, 0);
        Eigen::RowVector2d cartPoint = polarToCartesian(polarPoint, robPos, robRotAngle);
        occPoints.conservativeResize(occPoints.rows() + 1, Eigen::NoChange);
        occPoints.row(occPoints.rows() - 1) = cartPoint;

        // Calculates free points using bresenham and stores them into freePoints
        Eigen::MatrixX2d bresenhamPoints = bresenham(robPos[0], robPos[1], cartPoint[0], cartPoint[1]);
        for (int j = 0; j < bresenhamPoints.rows(); j++)
        {
            freePoints.conservativeResize(freePoints.rows() + 1, Eigen::NoChange);
            freePoints.row(freePoints.rows() - 1) = bresenhamPoints.block<1, 2>(j, 0);
        }
    }
    // Returns both matrices
    return std::make_pair(occPoints, freePoints);
}

Eigen::MatrixX2d OccupancyGrid::bresenham(int robPosX, int robPosY, int x, int y)
{
    Eigen::Matrix<double, -1, 2, Eigen::RowMajor> points;
    // Scale coordinates down to grid dimensions for less calculation
    int x1 = robPosX / (MAP_WIDTH / GRID_WIDTH), y1 = robPosY / (MAP_WIDTH / GRID_WIDTH);
    int x2 = x / (MAP_WIDTH / GRID_WIDTH), y2 = y / (MAP_WIDTH / GRID_WIDTH);

    // Move endpoint towards robot to exclude lidar point
    if (x1 > x2)
        x2++;
    else
        x2--;

    if (y1 > y2)
        y2++;
    else
        y2--;

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        if (x1 == x2 && y1 == y2)
            break;

        int e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y1 += sy;
        }

        // Scales result back to original dimenstions and saves it onto points
        points.conservativeResize(points.rows() + 1, Eigen::NoChange);
        points.row(points.rows() - 1) = Eigen::RowVector2d{x1 * (MAP_WIDTH / GRID_WIDTH), y1 * (MAP_WIDTH / GRID_WIDTH)};
    }
    return points;
}

Eigen::RowVector2d OccupancyGrid::polarToCartesian(const Eigen::RowVector2d &polarPoint, const Eigen::RowVector2d &robPos, double robRotAngle)
{
    Eigen::RowVector2d cartPoint;

    // Adds the robots rotation angle to align the axis of the scan data with the global axis
    double theta = polarPoint[0] + robRotAngle;
    double r = polarPoint[1];

    // Convert to Cartesian, add RobPos and round to int
    cartPoint[0] = round(r * cos(theta)) + robPos[0];
    cartPoint[1] = round(r * sin(theta)) + robPos[1];

    if (cartPoint[0] >= MAP_WIDTH || cartPoint[1] >= MAP_HEIGHT)
    {
        cartPoint = robPos;
    }
    if (cartPoint[0] < 0 || cartPoint[1] < 0)
    {
        cartPoint = robPos;
    }

    return cartPoint;
}
