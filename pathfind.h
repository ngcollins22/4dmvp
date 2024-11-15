#include "ot.h"
#include <math.h>


/*
    Header file containing functions and utilites to pathfind
*/

typedef struct Point {
    float x, y, z;
} point_t;

enum Maneuver {
    CRUISE,
    ACCELERATE,
    DECELERATE,
    LEFT,
    RIGHT,
    CLIMB,
    DESCEND,
    YETTOMOVE
};

/*
    Discrete state values the vehicle can assume
*/
typedef struct State {
    point_t position;
    float velocity;
    float heading;
    float time;

    state_t *previous;
    enum Maneuver maneuver;

    float h;
    float g;

} state_t;

/*
    Defines a vehicle performance profile
*/
typedef struct Vehicle {
    int R; // vehicle geofence radiues, in units of chi
    int vpref; // vehicle preferred (max) speed, in units of chi/tau
    int vstall; // vehicle minimum speed, in units of chi/tau
    float vdot; // accerelation/decceleration rate of the vehicle, in units of chi/tau^2
    float hdot; // rate of climb/descend of vehicle, in units of chi/tau
    float psidot; // rate of change of heading, rad/tau
} vehicle_t;

typedef struct Path {
    //Setup portion
    int id;
    vehicle_t vehicle;
    point_t startPoint;
    point_t endPoint;
    int ideal_tau_start;
    int solved;

    state_t* solution;    //list of states that is ultimately the solution

    float initialh;
    float finalg;
    float timeToSolve_ms;
} path_t;

typedef struct Heap {
    state_t** arr;
    int size;
    int capacity;
} heap_t;

/*
    Function to return a list of neighbors.
    Does not check for validity/traversibility.
*/

//fix ^ operator issue

extern heap_t* kinematicNeighbors(state_t* state, vehicle_t vehicle); //done?

extern void pathFind(map_t *map, path_t *path);

extern int boundsCheck(map_t *map, state_t *state); //done

extern int spaceCheck(map_t *map, state_t *state, vehicle_t vehicle); //not done

extern int closeEnough(map_t *map, state_t* a, point_t b); //done

extern void calcCosts(state_t *state, vehicle_t vehicle, point_t endPoint); //done

extern state_t* initialState(point_t startPoint, point_t endPoint, int t0, vehicle_t vehicle_t); //done

extern heap_t* trimNeighbors(map_t *map, heap_t *h, vehicle_t vehicle); //done (depends on others)?

extern void printPath(path_t path); //needs update

extern void freePath(path_t path);

extern void addPathToMap(map_t *map, path_t* path);

extern void spaceClaim(map_t *map, state_t *state);

extern heap_t* createHeap(int capacity);

extern void insert(heap_t *h, state_t* value);

extern state_t* extract(heap_t *h);

extern void printHeap(heap_t *h); //not done

extern void swap(state_t** a, state_t **b);

extern float f_value(state_t* state);

extern void freeHeap(heap_t *h);