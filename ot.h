#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
    Header file containing data structures of maps and occupancy tensors
*/

typedef struct OccupancyTensor {
    uint8_t* occupancymap; //data representation of the occupancy, compressed to avoid memory problems
    int W, L, H; //dimensions of the physical space
    int tf; //dimensions of the time space
    /*
        like [x][y][z][t] -> occupied or not (maybe probability?)
    */
} ot_t;

/*
    A map has dimensions and contains an array representing it's tiles. 
*/
typedef struct Map {
    int W, L, H; //Dimensions of the map, in unitWidth units

    time_t referenceTime; // UTC "start of day"
    time_t startTime; // UTC start of this given map (should typically be now)

    int timeStep_s; // integer number of seconds between timesteps
    int unitWidth_m; // integer number of meters as length of cube space step
    ot_t *occupancytensor; // 4D tensor representing occupancy over time

} map_t;

/*
    Create a new blank map with given spatial dimensions
        Assumed reference time to be zero utc, same with startTime

*/
extern map_t *emptyMap(int w, int l, int h, int tf);

/*
    Pull the value of a map at spatial and temporal coordinates
        Returns -1 if the index does not exist
*/
extern int pull(map_t *map, int x, int y, int z, int t);

/*
    Create a new tensor with given dimensions
*/
extern ot_t *createTensor(int w, int l, int h, int tf);

extern void freeMap(map_t *map);

extern void freeTensor(ot_t *ot);

extern int getIndex(ot_t *ot, int x, int y, int z, int t);

extern int getValueAt(ot_t *ot, int index);

extern void setValueAt(ot_t *ot, int index, int value);

extern ot_t *copyTensor();

extern void exportMap(map_t *map, FILE* out);

extern void extendTimeHorizon(ot_t *ot, int n);

extern void leftTruncate(ot_t *ot);

/*
    TODO: I need a way to
        Update a tensor with a new path that's just been planned
        left-truncate the history off the tensor & update map's parameters
*/