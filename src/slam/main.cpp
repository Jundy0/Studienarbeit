#include <vector>

#include "Eigen/Dense"

#include "particle.h"

Eigen::MatrixX2d getDataFromFile(std::string filePath);

int main()
{
    Particle particle;

    Eigen::MatrixX2d scan01 = getDataFromFile("../scans/scan0001.csv");
    Eigen::MatrixX2d scan02 = getDataFromFile("../scans/scan0002.csv");
    Eigen::MatrixX2d scan03 = getDataFromFile("../scans/scan0003.csv");
    Eigen::MatrixX2d scan04 = getDataFromFile("../scans/scan0004.csv");
    Eigen::MatrixX2d scan05 = getDataFromFile("../scans/scan0005.csv");

    particle.update(scan01, scan02);
    particle.update(scan02, scan03);
    particle.update(scan03, scan04);
    particle.update(scan04, scan05);
    particle.visualizeGridMap();

    return 0;
}

Eigen::MatrixX2d getDataFromFile(std::string filePath)
{
    std::ifstream file(filePath);
    if (!file.is_open())
        throw std::runtime_error("Could not open file");

    std::string line, word;
    Eigen::Matrix<double, -1, 2, Eigen::RowMajor> result;

    while (getline(file, line))
    {
        std::vector<double> values;
        std::stringstream ss(line);

        while (getline(ss, word, ','))
        {
            values.push_back(stod(word));
        }

        result.conservativeResize(result.rows() + 1, Eigen::NoChange);
        result.row(result.rows() - 1) = Eigen::RowVector2d{values[0], values[1]};
    }
    return result;
}
