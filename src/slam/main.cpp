#include "include/occupancy_grid.h"

using namespace std;

int main()
{
    OccupancyGrid grid;
    grid.updateProbMap("C:/Users/sej1grb/Desktop/DHBW/Studienarbeit/src/slam/log.csv");
    grid.visualize();
    
    return 0;
}