#include "include/occupancy_grid.h"

using namespace std;

int main()
{
    OccupancyGrid grid;
    grid.updateCells("./log.csv");
    
    return 0;
}
