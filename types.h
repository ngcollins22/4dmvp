// Header file for all the types used here

/*
    Status enum for tracking if a node is empty, permanently blocked, or temporarily occupied
*/
typedef enum Status {
    FREE,
    OBSTACLE,
    OCCUPIED,
} status_t;


typedef struct Node node_t;

/*
    A node is a data structure representing a tile in a map
*/
typedef struct Node {
    status_t status;
    int occupiedBy;
    float hcost, gcost;
    node_t *prev;
    int index;
    int x, y;
} node_t;

/*
    A map has dimensions and contains an array representing it's tiles. 
*/
typedef struct Map {
    node_t* grid;
    int w;
    int l;
    float density;
} map_t;



/*
    Data structure to hold a flight plan
*/
typedef struct FlightPlan {
    int start, end;
    int id;
    int wasSuccess;
    double timeToSolve;
} flightplan_t;