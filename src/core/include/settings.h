#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#define MAP_WIDTH 2000  // Width of Map
#define MAP_HEIGHT 2000 // Height of Map

#define GRID_WIDTH 500  // Width of Grid
#define GRID_HEIGHT 500 // Height of Grid

#define VEHICLE_RADIUS 10          // The Radius of the Vehicle (in cm) ~20-25cm
#define VEHICLE_MAX_ROT_ANGLE 0.18 // Max. Rotational Angle of the Vehicle (in rad) ~10-15°

#define PROB_OCC 2                    // Problably Occupied
#define PROB_FREE -1                  // Probably Free
#define PROB_OCC_THRES PROB_OCC + 1   // Threshold for Occupied
#define PROB_FREE_THRES PROB_FREE - 1 // Threshold for Free
#define INFLATED 1                    // Value for Infalted Obstacles

#define DELTA_OCC 0.75  // Delta Value for Occupied
#define DELTA_FREE -0.4 // Delta Value for Free

#define MIN_ICP_DISTANCE 300
#define MAX_ICP_DISTANCE 1500

#define ICP_FITNESS_THRESHOLD 10
#define ICP_MAX_NR_CORRECTIONS 15

#endif // __SETTINGS_H__
