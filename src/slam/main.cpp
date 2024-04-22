#include <vector>

#include "lib/eigen/Eigen/Dense"

#include "include/particle.h"

using namespace std;

Eigen::MatrixX2d getDataFromFile(string filePath);

int main()
{
    Particle particle;

    Eigen::MatrixX2d firstScan = getDataFromFile("./data/log_1.txt");
    Eigen::MatrixX2d secondScan = getDataFromFile("./data/log_2.txt");

    particle.update(firstScan, secondScan);

    return 0;
};

Eigen::MatrixX2d getDataFromFile(string filePath) {
    ifstream file(filePath);
    if(!file.is_open()) throw std::runtime_error("Could not open file");
    
    string line, word;
    Eigen::Matrix<double, -1, 2, Eigen::RowMajor> result;
    
    while (getline(file, line)) {
        
        std::vector<double> values;
        stringstream ss(line);

        while (getline(ss, word, ',')) {
        values.push_back(stod(word)); 
        }

        result.conservativeResize(result.rows()+1,Eigen::NoChange);
        result.row(result.rows()-1) = Eigen::RowVector2d{values[0], values[1]};
    }
    return result;
};