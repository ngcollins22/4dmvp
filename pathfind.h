#include "ot.h"
#include <math.h>


/*
    Header file containing functions and utilites to pathfind
*/

typedef struct State state_t;

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

    state_t* previous;
    enum Maneuver maneuver;

    float h;
    float g;
    int isFinalSolution;

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
    vehicle_t vehicle;
    point_t startPoint;
    point_t endPoint;
    int ideal_tau_start;


    //post-solution portion
    int id;
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


/*
    Returns a list of the kinematically feasible neighbor states.

    @param state The base state to check
    @param vehicle The vehicle performance profile
*/
extern heap_t* kinematicNeighbors(state_t* state, vehicle_t vehicle); //done?

/*
    Main pathfinding algorithm. Assumes that path vehicle, start and end points, and ideal start time are already defined.
    Updates the path with solution (or not) and other characteristics of the solution.

    @param map Occupancy map of the space
    @param path The path to solve
*/
extern void pathFind(map_t *map, path_t *path);

/*
    Simple check on state to see if map bounds have been exceeded.

    @param map Occupancy map of the space
    @param state The state to check
*/
extern int boundsCheck(map_t *map, state_t *state); //done


/*
    Complex check of whether a state is permissible based on existing occupancy in the map

    @param map Occupancy map of the space
    @param state The state to check
    @param vehicle The performance profile of the vehicle
*/
extern int spaceCheck(map_t *map, state_t *state, vehicle_t vehicle); //not done


/*
    Cartesian distance between two points

    @param p1 First point
    @param p2 Second point
*/
extern float dist(point_t p1, point_t p2);

/*
    Utility function to add small offsets to a point. Does not modify original point.

    @param p Base point
    @param dx x-offset
    @param dy y-offset
    @param dz z-offset
*/
extern point_t add(point_t p, float dx, float dy, float dz);

/*
    Utility function used in pathFind() to check if a state is close enough to the final solution to be declared the end.

    @param a State to check
    @param b Point to check against
*/
extern int closeEnough(state_t* a, point_t b); //done

/*
    Utility function used in pathFind() to check if two states are close enough

    @param a State to check
    @param b State to check
*/
extern int closeEnoughState(state_t* a, state_t* b); //done


/*
    Complex function that calculates the costs of a state and updates it with them.

    @param state The state to calculate & update
    @param endPoint The final/goal state
    @param vehicle The vehicle performance profile

*/
extern void calcCosts(state_t *state, point_t endPoint, vehicle_t vehicle); //done

extern state_t* initialState(point_t startPoint, point_t endPoint, int t0, vehicle_t vehicle_t); //done

extern heap_t* trimNeighbors(map_t *map, heap_t *h, vehicle_t vehicle); //done (depends on others)?

extern void printPath(path_t *path);

extern void printPoint(point_t point);

extern void freePath(path_t* path);

extern void freeStateSequence(state_t *state);

extern void writePath(map_t *map, path_t *path);

extern void spaceClaim(map_t *map, state_t *state, vehicle_t vehicle);

extern heap_t* createHeap(int capacity);

extern void insert(heap_t *h, state_t* value);

extern state_t* extract(heap_t *h);

extern void printHeap(heap_t *h); //not done

extern void swap(state_t** a, state_t **b);

extern float f_value(state_t* state);

extern void freeHeap(heap_t *h);

extern void cleanHeap(heap_t *h);

extern void trimBranch(state_t *state);

extern void printStateSequence(state_t *state);

extern const char *maneuver_to_string(int maneuver);

extern void printState(state_t *state);

extern void cleanList(state_t **list, int size);