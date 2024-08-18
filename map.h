/*
    Header file containing implemented functions related to map objects

*/
#include <node.h>
#include <stdio.h>

/*
    A map has dimensions and contains an array representing it's tiles
*/
typedef struct Map {
    node_t* grid;
    int w;
    int l;
} map_t;


/*
    Returns a pointer to a blank map with given length and width
*/
extern map_t *create_blank_map(unsigned int l, unsigned int w);

/*
    Populates a map with a number of obstacles
*/
extern void populate_with_obstacles(map_t* map, unsigned int numObstacles);

/*
    Get a node at a specific index. Returns null if index invalid
*/
extern node_t *getIndex(map_t* map, int n);

/* 
    Get a node at a specific x and y location. Returns null if index invalid
*/
extern node_t *getLocation(map_t* map, int x, int y);

