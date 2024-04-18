#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "../lib/eigen/Eigen/Dense"

using namespace std;

class OccupancyGrid
{

public:
    OccupancyGrid();
    void updateProbMap(string filePath);
    void visualize();

private:
    double probOcc;
    double probFree;
    int gridWidth;
    int gridHeight;

    Eigen::MatrixXd probMap;

    Eigen::MatrixX2d getDataFromFile(string filePath);
    pair<Eigen::MatrixX2i, Eigen::MatrixX2i> getPoints(string filePath);
    Eigen::MatrixX2i bresenham(int robPosX, int robPosY, int x, int y);
    Eigen::RowVector2i polarToCartesian(Eigen::RowVector2d polarPoint, Eigen::RowVector2i robPos);
};
