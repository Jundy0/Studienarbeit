#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <matplot/matplot.h>

using namespace std;

class OccupancyGrid {
    public:
        OccupancyGrid() {
            // Sensor characteristic: Min and Max ranges of the beams
            double Zmax = 5000, Zmin = 170;
            // Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
            double l0 = 0, locc = 0.4, lfree = -0.4;
            // Grid dimensions
            double gridWidth = 100, gridHeight = 100;
            // Map dimensions
            double mapWidth = 10000, mapHeight = 10000;
            // Defining an l vector to store the log odds values of each cell
            vector< vector<double> > cells(mapWidth/gridWidth, vector<double>((mapHeight/gridHeight), 0.0));
        }

        void updateCells(string filePath) {
            vector< vector< pair<int, int> > > allPoints = getPoints(filePath);

            vector< pair<int, int> > occPoints = allPoints[0];
            vector< pair<int, int> > freePoints = allPoints[1];

            for (int i = 0; i < (occPoints.size()/occPoints[0].size()), x++) {
                int x = occPoints[i][0];
                int y = occPoints[i][1];
                if (cells[x][y] < 1) += 0.1;
            }

            for (int i = 0; i < (freePoints.size()/freePoints[0].size()), x++) {
                int x = freePoints[i][0];
                int y = freePoints[i][1];
                if (cells[x][y] > -1) -= 0.1;
            }

        }

        void visualize() {

        }

    private:
        vector< vector<double> > getDataFromFile(string filePath) {
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

        vector< vector< pair<int, int> > > getPoints(string filePath) {
            //Robot position
            pair<int, int> robPos = {50, 50};
            
            vector< pair<int, int> > occPoints;
            vector< pair<int, int> > freePoints;
            
            vector< vector<double> > data = getDataFromFile(filePath);
            
            for (int i = 0; i < (data.size()/data[0].size()), i++) {
                vector<double> polarPoint = data[i];
                pair<int, int> cartPoint = polarToCartesian(polarPoint);
                occPoints.push_back(cartPoint);
                
                vector< pair<int, int> > bresenhamPoints = bresenham(robPos[0], robPos[1], cartPoint[0], cartPoint[1]);
                for (int j = 0; j < (bresenhamPoints.size()/bresenhamPoints[0].size()), j++) {
                    freePoints.push_back(bresenhamPoints[j]);
                }
            }

            vector< vector< pair<int, int> > > result;
            result.push_back(occPoints);
            result.push_back(freePoints);

            return result;
        }

        vector< pair<int, int> > bresenham(int x1, int y1, int x2, int y2) {
            int m_new = 2 * (y2 - y1); 
            int slope_error_new = m_new - (x2 - x1);
            
            vector< pair<int, int> > points;
            
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

        pair<int, int> polarToCartesian(vector<double> polarPoint) {
            pair<int, int> cartPoint;
            
            double theta = polarPoint[0];
            double r = polarPoint[1]*100;

            cartPoint[0] = round(r * cos(theta))+50;
            cartPoint[1] = round(r * sin(theta))+50;

            return cartPoint;
        }




};

int main()
{

    OccupancyGrid grid;
    grid.updateCells("./log.csv");
    
    return 0;
}