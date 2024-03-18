#include <matplot/matplot.h>
#include "../include/occupancy_grid.h"

using namespace std;

OccupancyGrid::OccupancyGrid() {
    // Sensor characteristic: Min and Max ranges of the beams
    Zmax = 5000;
    Zmin = 170;
    // Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
    locc = 0.5;
    lfree = -0.5;
    // Grid dimensions
    gridWidth = 100;
    gridHeight = 100;
    // Map dimensions
    mapWidth = 10000;
    mapHeight = 10000;
    // Defining an l vector to store the log odds values of each cell
    cells.resize(gridWidth, vector<double>(gridHeight, 0.0));
};

void OccupancyGrid::updateCells(string filePath) {
    vector< vector< pair<int, int> > > allPoints = getPoints(filePath);

    vector< pair<int, int> > occPoints = allPoints[0];
    vector< pair<int, int> > freePoints = allPoints[1];

    for (int i = 0; i < occPoints.size(); i++) {
        int x = occPoints[i].first;
        int y = occPoints[i].second;
        if (cells[x][y] < 1) cells[x][y] += 0.1;
    }

    for (int i = 0; i < freePoints.size(); i++) {
        int x = freePoints[i].first;
        int y = freePoints[i].second;
        if (cells[x][y] > -1) cells[x][y] -= 0.03;
    }

}

void OccupancyGrid::visualize() {
    const string BLACK_COLOR = "\033[40m";
    const string GREEN_COLOR = "\033[42m";
    const string WHITE_COLOR = "\033[47m";
    const string DEFAULT_COLOR = "\033[0m";

    cout << WHITE_COLOR;

    for (int i = 0; i < gridWidth; i++) {
        for (int j = 0; j < gridHeight; j++) {
            if (cells[i][j] >= locc)
                cout << BLACK_COLOR << " " << WHITE_COLOR;
            else if (cells[i][j] <= lfree)
                cout << GREEN_COLOR << " " << WHITE_COLOR;
            else
                cout << " ";
        }
        cout << DEFAULT_COLOR << endl;
    }
}

vector< vector<double> > OccupancyGrid::getDataFromFile(string filePath) {
    ifstream file(filePath);
    if(!file.is_open()) throw std::runtime_error("Could not open file");
    
    string line, word;
    vector< vector<double> > result;
    
    
    while (getline(file, line)) {
        
        vector<double> values;
        stringstream ss(line);

        while (getline(ss, word, ',')) {
            values.push_back(stod(word));
        }

        result.push_back(values);
    }
    
    return result;
}

vector< vector< pair<int, int> > > OccupancyGrid::getPoints(string filePath) {
    //Robot position
    pair<int, int> robPos = {50, 50}; // Hardcode needs changing
    
    vector< pair<int, int> > occPoints;
    vector< pair<int, int> > freePoints;
    
    vector< vector<double> > data = getDataFromFile(filePath);
    
    for (int i = 0; i < data.size(); i++) {
        vector<double> polarPoint = data[i];
        pair<int, int> cartPoint = polarToCartesian(polarPoint, robPos);
        occPoints.push_back(cartPoint);
        
        vector< pair<int, int> > bresenhamPoints = bresenham(robPos.first, robPos.second, cartPoint.first, cartPoint.second);
        for (int j = 0; j < bresenhamPoints.size(); j++) {
            freePoints.push_back(bresenhamPoints[j]);
        }
    }

    vector< vector< pair<int, int> > > result;
    result.push_back(occPoints);
    result.push_back(freePoints);

    return result;
}

vector< pair<int, int> > OccupancyGrid::bresenham(int robPosX, int robPosY, int x, int y) {
    vector< pair<int, int> > points;
    int x1 = robPosX, y1 = robPosY;
    int x2 = x, y2 = y;
    
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

    while (true) {
        if (x1 == x2 && y1 == y2) 
            break;
        
        int e2 = 2 * err;
        
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }

        points.push_back({x1, y1});
    }

    
    
    return points;
}

pair<int, int> OccupancyGrid::polarToCartesian(vector<double> polarPoint, pair<int, int> robPos) {
    pair<int, int> cartPoint;
    
    double theta = polarPoint[0];
    double r = polarPoint[1] * 10 * 100; // Meter to Centimeter plus fitst digit after comma  

    cartPoint.first = round(r * cos(theta) / 10) + robPos.first;
    cartPoint.second = round(r * sin(theta) / 10) + robPos.second;

    return cartPoint;
};
