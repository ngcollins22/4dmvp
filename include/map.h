/*
    Header file containing implemented functions related to map objects

*/
#include "node.h"

/*
    A map has dimensions and contains an array representing it's tiles
*/
typedef struct Map {


} map_t;

extern map_t *create_blank_map(unsigned int l, unsigned int w);

extern void populate_with_obstacles(map_t* map, unsigned int numObstacles);

