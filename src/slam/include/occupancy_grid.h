#include <iostream>
#include <string>
#include <math.h>
#include <vector>

using namespace std;

class OccupancyGrid {
    
    public:
        OccupancyGrid();
        void updateCells(string filePath);
        void visualize();

    private:
        // Sensor characteristic: Min and Max ranges of the beams
        double Zmax;
        double Zmin;
        // Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
        double l0;
        double locc;
        double lfree;
        // Grid dimensions
        double gridWidth;
        double gridHeight;
        // Map dimensions
        double mapWidth;
        double mapHeight;
        // Defining an l vector to store the log odds values of each cell
        std::vector< std::vector<double> > cells;

        vector< vector<double> > getDataFromFile(string filePath);
        vector< vector< pair<int, int> > > getPoints(string filePath);
        vector< pair<int, int> > bresenham(int x1, int y1, int x2, int y2);
        pair<int, int> polarToCartesian(vector<double> polarPoint, pair<int, int>);
};
