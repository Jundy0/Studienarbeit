#include <matplot/matplot.h>
#include "../include/occupancy_grid.h"

using namespace std;
OccupancyGrid::OccupancyGrid() {
    // Sensor characteristic: Min and Max ranges of the beams
    Zmax = 5000;
    Zmin = 170;
    // Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
    l0 = 0;
    locc = 0.4;
    lfree = -0.4;
    // Grid dimensions
    gridWidth = 100;
    gridHeight = 100;
    // Map dimensions
    mapWidth = 10000;
    mapHeight = 10000;
    // Defining an l std::vector to store the log odds values of each cell
    cells.resize(mapWidth/gridWidth, std::vector<double>(mapHeight/gridHeight));
};

void OccupancyGrid::updateCells(string filePath) {
    std::vector< std::vector< pair<int, int> > > allPoints = getPoints(filePath);

    std::vector< pair<int, int> > occPoints = allPoints[0];
    std::vector< pair<int, int> > freePoints = allPoints[1];

    for (int i = 0; i < (sizeof(occPoints)/sizeof(occPoints[0])); i++) {
        int x = occPoints[i].first;
        int y = occPoints[i].second;
        if (cells[x][y] < 1) cells[x][y] += 0.1;
    }

    for (int i = 0; i < (sizeof(freePoints)/sizeof(freePoints[0])); i++) {
        int x = freePoints[i].first;
        int y = freePoints[i].second;
        if (cells[x][y] > -1) cells[x][y] -= 0.1;
    }

}

void OccupancyGrid::visualize() {

}

std::vector< std::vector<double> > OccupancyGrid::getDataFromFile(string filePath) {
    ifstream file(filePath);
    if(!file.is_open()) throw std::runtime_error("Could not open file");
    
    string line, word;
    std::vector< std::vector<double> > result;
    
    
    while (getline(file, line)) {
        
        std::vector<double> values;
        stringstream ss(line);

        while (getline(ss, word, ',')) {
            values.push_back(stod(word));
        }

        result.push_back(values);
    }
    
    return result;
}

std::vector< std::vector< pair<int, int> > > OccupancyGrid::getPoints(string filePath) {
    //Robot position
    pair<int, int> robPos = {50, 50};
    
    std::vector< pair<int, int> > occPoints;
    std::vector< pair<int, int> > freePoints;
    
    std::vector< std::vector<double> > data = getDataFromFile(filePath);
    
    for (int i = 0; i < (data.size()/data[0].size()); i++) {
        std::vector<double> polarPoint = data[i];
        pair<int, int> cartPoint = polarToCartesian(polarPoint);
        occPoints.push_back(cartPoint);
        
        std::vector< pair<int, int> > bresenhamPoints = bresenham(robPos.first, robPos.second, cartPoint.first, cartPoint.second);
        for (int j = 0; j < (sizeof(bresenhamPoints)/sizeof(bresenhamPoints[0])); j++) {
            freePoints.push_back(bresenhamPoints[j]);
        }
    }

    std::vector< std::vector< pair<int, int> > > result;
    result.push_back(occPoints);
    result.push_back(freePoints);

    return result;
}

std::vector< pair<int, int> > OccupancyGrid::bresenham(int x1, int y1, int x2, int y2) {
    int m_new = 2 * (y2 - y1); 
    int slope_error_new = m_new - (x2 - x1);
    
    std::vector< pair<int, int> > points;
    
    for (int x = x1, y = y1; x <= x2; x++) {
        pair<int, int> point = {x, y};
        points.push_back(point);

        // Add slope to increment angle formed 
        slope_error_new += m_new; 

        // Slope error reached limit, time to 
        // increment y and update slope error. 
        if (slope_error_new >= 0) { 
            y++; 
            slope_error_new -= 2 * (x2 - x1); 
        } 
    }

    return points;
}

pair<int, int> OccupancyGrid::polarToCartesian(std::vector<double> polarPoint) {
    pair<int, int> cartPoint;
    
    double theta = polarPoint[0];
    double r = polarPoint[1]*100;

    cartPoint.first = round(r * cos(theta))+50;
    cartPoint.second = round(r * sin(theta))+50;

    return cartPoint;
};
