#include "ot.h"
#include <math.h>

/*
    Header file containing functions and utilites to pathfind
*/


const float CHI; //meters
const float TAU; //seconds
const float G_CANONCIAL; // Canonical g value


/*
    Important struct used in the algorithm. Contains the information about a state.
*/
typedef struct State state_t;

/*
    Simple struct used to represent a point in 3D space.
*/
typedef struct Point {
    float x, y, z;
} point_t;

/*
    Simple enum to distinguish maneuvers.
*/
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

typedef struct State {
    point_t position; // 3D position of the state
    float velocity; // Velocity
    float heading; // Heading (radians)
    float time; // Time (in temporal epochs)

    state_t* previous; // Pointer to previous state
    enum Maneuver maneuver; // Enum of the maneuver between the previous state and this one

    float h; // Heuristic cost
    float g; // Path cost
    int isFinalSolution; // Flag set if this state ends up in the final solution.
} state_t;

/*
    Defines a vehicle performance profile
*/
typedef struct Vehicle {
    float W; //total vehicle weight
    float LD; //lift to drag ratio
    float hoverPower; //power required to hover
    int R; // vehicle geofence radiues, in units of chi
    int vpref; // vehicle preferred (max) speed, in units of chi/tau
    int vstall; // vehicle minimum speed, in units of chi/tau
    float vdot; // accerelation/decceleration rate of the vehicle, in units of chi/tau^2
    float hdot; // rate of climb/descend of vehicle, in units of chi/tau
    float psidot; // rate of change of heading, rad/tau
} vehicle_t;

typedef struct Path {
    //Setup portion - defined before pathfind is called
    vehicle_t vehicle; // vehicle performance profile
    point_t startPoint; // path start point
    point_t endPoint; // path end point
    int ideal_tau_start; // path request time/ideal start time


    //post-solution portion - updates after pathfind exits
    int id; // unique id assigned
    int solved; // 0 - failed, 1 - solved

    /*
        Pointer to the final node in the solution. Linked-list traversal provides full solution.
        **NOTE: Null if the solution was not found (rare)
    */
    state_t* solution;


    float initialh; // Initial heuristic, for stats
    float finalg; // Final path cost, for stats
    float timeToSolve_ms; // Time it took machine to find the solution, or fail
} path_t;

/*
    Min-heap used to improve the speed of the pathfinding algorithm.

*/
typedef struct Heap {
    state_t** arr; // Array to store the states in the heap
    int size; // Current size
    int capacity; // Maximum size
} heap_t;

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
extern int closeEnough(state_t* a, point_t b); 

/*
    Utility function used in pathFind() to check if two states are similiar enough
    to call them identical and trigger rejection to avoid re-exploration

    @param a State to check
    @param b State to check
*/
extern int closeEnoughState(state_t* a, state_t* b);

/*
    Complex function that calculates the costs of a state and updates it with them.

    @param state The state to calculate & update
    @param endPoint The final/goal state
    @param vehicle The vehicle performance profile
*/
extern void calcCosts(state_t *state, point_t endPoint, vehicle_t vehicle); 


/*
    Special function used to initialize the pathfinding algorithm and generate the head node

    @param startPoint The initial point of the pathfinding algorithm.
    @param endPoint The goal point of the pathfinding algorithm.
    @param t0 The ideal/requested initial time to plan the path
    @param vehicle The performance profile of the vehicle

    @return The initial state to be used in the pathfinding algorithm
*/
extern state_t* initialState(point_t startPoint, point_t endPoint, int t0, vehicle_t vehicle); 


/*
    Function to evaluate a list of neighbors and determine if it's feasible given the
    current occupancy tensor of the state

    @param map The map containing the occupancy tensor and other metadata of the map
    @param h A heap of neighbors to be checked and trimmed. Any rejected neighbors are freed.
    @param vehicle The performance profile of the vehicle using the algorithm

    @return A new heap containing all the neighbors that pass each check.
*/
extern heap_t* trimNeighbors(map_t *map, heap_t *h, vehicle_t vehicle); 


/*
    Utility function to print out the information about a path.
    Assumes the path has been solved/attempted, can handle both success and failure.

    @param path The path to print out. Pathfind must be called on path before using this function.
*/
extern void printPath(path_t *path);


/*
    Utility function to print out the information about a simple point struct.

    @param point The point to print information about.
*/
extern void printPoint(point_t point);


/*
    Safely frees a path. Frees metadata and every state_t struct contained in the solution (if it exists).

    @param path Pointer to path to free
*/
extern void freePath(path_t* path);


/*
    Recursively frees a state and it's previous states like a linked list.

    @param state State struct representing the root of the linked list.
*/
extern void freeStateSequence(state_t *state);

/*
    Function to write the information about path into the occupancy tensor. 
    Generates the geofencing information and updates the map accordingly.

    @param map The map to update
    @param path The path to write into the map
*/
extern void writePath(map_t *map, path_t *path);

/*
    Function to write a specific state's geofencing into the map. 
    Considers the vehicle's path from the previous state to this state.

    @param map The map to update with the geofencing.
    @parma state The state to geofence TO, considers the previous state as well.
    @param vehicle The vehicle performance profile.

*/
extern void spaceClaim(map_t *map, state_t *state, vehicle_t vehicle);

/*
    Creates a minheap data structure. Stores state_t structs.

    @param capacity Max capacity of heap.
*/
extern heap_t* createHeap(int capacity);

/*
    Insert a new state into a heap. Maintains sorting by lowest f-cost at the top.

    @param h The heap to update
    @param value The state struct to insert.
*/
extern void insert(heap_t *h, state_t* value);

/*
    Extract the minimum f-cost state from a heap. Maintains sorting by lowest f-cost at the top.

    @param h The heap to extract from

    @return The lowest f-cost state in the list.
*/
extern state_t* extract(heap_t *h);

/*
    Utility function to print the information in a heap. Not nessecarily safe - used only for debugging.

    @param h The heap to print
*/
extern void printHeap(heap_t *h); //not done

/*
    Utility function to swap two states. Used in minheap functions.

    @param a First state pointer pointer
    @param b Second state pointer pointer
*/
extern void swap(state_t** a, state_t **b);

/*
    Utility function to calculate the total cost of a state. 

    @param state State of interest

    @return The f-cost of the state. Sum of g and h costs.
*/
extern float f_value(state_t* state);

/*
    Safely frees a heap and its array. Does NOT free the states in the heap. 
    Cleanheap should be called first.

    @param h Heap to free.
*/
extern void freeHeap(heap_t *h);

/*
    Frees the contents of a heap's array. Does not respect isFinalSolution tag on states. 
    Should only be used once all solution states are extracted from the heap.

    @param h Heap to clean.
*/
extern void cleanHeap(heap_t *h);

/*
    Iteratively prints state sequences linked-list style. Prints DOWN.

    @param state Root state

*/
extern void printStateSequence(state_t *state);

/*
    Utility function to turn a maneuver into a string. Returned strings need not be freed.

    @param maneuver Int/Enum of maneuver as defined above.
    
    @return Constant string.
*/
extern const char *maneuver_to_string(int maneuver);

/*
    Utility function to print out a state's information.

    @param state The state to print out.
*/
extern void printState(state_t *state);

/*
    Function to free the contents of a simple array.

    @param list List of state pointers.
    @param size Size of list.
*/
extern void cleanList(state_t **list, int size);

/*
    Utility function to reset the global id counter used to identify paths.
*/
extern void resetGlobalIdCounter();

/*
    Exports the information about a path and its solution to a file named "path#.csv" where # is the id assigned to the path.

    @param path The path to export.
*/
extern void exportPath(path_t *path);

/*
    Prints out just the major statistics of a path to a file. Omits the full path. Used for collecting statistics.

    @param out The file pointer to print to
    @param path The path to print out.
*/
extern void exportPathShort(FILE* out, path_t *path);