#include <iostream>
#include <fstream> 
#include <tuple>
#include <vector>
#include <string>
#include <sstream>
#include "../slam/lib/icp/Eigen/Dense"

void readPolarCoordinates(std::vector<double> &distances, std::vector<double> &angles)
{
    double trainsample;
    std::ifstream file("../slam/data/coordiantes.csv");
if (file.is_open()) 
{
    std::string line;
    while (std::getline(file, line)) 
    {
        // using printf() in all tests for consistency

        std::string delimiter = ",";

        size_t pos = 0;
        int occurence = 0;
        std::string token;
        while ((pos = line.find(delimiter)) != std::string::npos) 
        {
            token = line.substr(0, pos);
            line.erase(0, pos + delimiter.length());
            if(occurence == 0)
            {
                double distance = std::stod(token);
                if(distance < 0)
                {
                    distance *= -1;
                }
                distances.push_back(distance);
                occurence ++;
            }
            if(occurence == 1)
            {
                angles.push_back(std::stod(token));
                occurence = 0;
            }
        }

    }
    file.close();
}
else
{
std::cout << "trouble loading file" << std::endl;

}

}

Eigen::MatrixX2d getDataFromFile() {
    std::ifstream file("../slam/data/coordiantes.csv");
    if(!file.is_open()) throw std::runtime_error("Could not open file");
    
    std::string line, word;
    Eigen::Matrix<double, -1, 2, Eigen::RowMajor> result;
    
    while (getline(file, line)) {
        
        std::vector<double> values;
        std::stringstream ss(line);

        while (getline(ss, word, ',')) {
           values.push_back(stod(word)); 
        }

        result.conservativeResize(result.rows()+1,Eigen::NoChange);
        result.row(result.rows()-1) = Eigen::RowVector2d{values[0], values[1]};
    }
    return result;
}