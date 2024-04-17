#include <iostream>
#include <fstream> 
#include <tuple>
#include <vector>
#include <string>
#include <sstream>

void readPolarCoordinates(std::vector<double> &distances, std::vector<double> &angles)
{
    double trainsample;
    std::ifstream file("/home/janik/janik/Studienarbeit/src/icp-test/coordiantes.csv");
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