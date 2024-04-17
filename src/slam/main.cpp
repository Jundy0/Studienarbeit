#include "include/occupancy_grid.h"

using namespace std;

int main()
{
    OccupancyGrid grid;
    grid.updateProbMap("./log.csv");
    grid.visualize();
    
    return 0;
}
