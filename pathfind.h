#include "ot.h"
#include <math.h>


/*
    Header file containing functions and utilites to pathfind
*/

typedef struct Node node_t;

typedef struct Point {
    int x, y, z;
} point_t;

typedef struct Waypoint {
    point_t location;
    float time;
} wp_t;

typedef struct Node {
    point_t point;
    float gcost;
    float t; //here, time is allowed to be non-integer
    float hcost;
    node_t *prev;
} node_t;

typedef struct Path {
    int solved;
    int pathLength;
    wp_t* waypoints;
    float tf;
    int timeToSolvems;
} path_t;

/*
    Function to return a list of neighbors.
    Does not check for validity/traversibility.
*/
extern node_t** getNeighbors(map_t *map, node_t* node, point_t goal, int* numberOfNeighbors);

extern path_t pathFind(map_t *map, point_t start, point_t end);

extern int spaceCheck(map_t *map, node_t* node);

extern node_t* makeNode(node_t *prev, point_t point, float dg, float dt, point_t goal);

extern node_t* makeStarterNode(point_t point, point_t goal);

extern point_t point(int x, int y, int z);

extern float dist(point_t p1, point_t p2); 

extern float fcost(node_t *node);

extern void removeFromList(node_t ***list, int *size, int index);

extern void addToList(node_t ***list, int *size, node_t *node);

extern int contains(node_t **list, int *size, node_t *node); 

extern int match(point_t p1, point_t p2);

extern wp_t nodeToWaypoint(node_t *node);

extern void printPath(path_t path);

extern void freeList(node_t **list, int size);

extern void freePath(path_t path);

extern void addPathToTensor(ot_t *ot, path_t* path);

extern void addWaypointToTensor(ot_t *ot, wp_t wp);