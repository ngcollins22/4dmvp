#include "types.h"
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>



/*
    Choose a random starting and ending location for a vehicle
*/
extern void chooseRandom(flightplan_t *plan, map_t *map, int id);

/*
    
*/
extern int pathfind(flightplan_t *plan, map_t *map, double *compTime);


extern void printplan(flightplan_t *plan, map_t *map);

/*
    Helper function to determine is a node is free
*/
extern int isfree(node_t *node);

/*
    Helper function to create a map from a file definition beforehand
*/
//extern map_t* mapFromFile(FILE *in);

/*
    Helper function to read a list of flights from a file
*/
extern void getPosition(map_t *map, int n, int *x, int *y);


extern void getIndex(map_t *map, int x, int y, int *n);


extern void calc_heuristic(map_t *map, node_t *nodeOfInterest, node_t *goalNode);


// find neighbors
// find geofence

/*
    Header file containing implemented functions related to map objects

*/


/*
    Returns a pointer to a blank map with given length and width
*/
extern map_t *create_blank_map(unsigned int l, unsigned int w);

/*
    Populates a map with a number of obstacles non-deterministically
*/
extern void populate_with_obstacles(map_t* map, float difficulty);

/*
    Get a node at a specific index. Returns null if index invalid
*/
extern node_t *pullIndex(map_t* map, int n);


/*
    Get the number of tiles in a map
*/
extern int getSize(map_t* map);

/* 
    Get a node at a specific x and y location. Returns null if index invalid
*/
extern node_t *getLocation(map_t* map, int x, int y);


/*
    Write a map to a file
*/
extern void writeToFile(map_t* map, FILE *out);


/*
    Debug print to std out
*/

extern void displayMap(map_t* map);
extern void displayNode(node_t *node);


/*
    free grid
*/
extern void freeGrid(map_t *map);

extern int fcost(node_t *node);


