/*
    Header file containing implemented functions related to node objects

*/

/*
    Status enum for tracking if a node is empty, permanently blocked, or temporarily occupied
*/
typedef enum Status {
    OBSTACLE,
    FREE,
    OCCUPIED
} status_t;

/*
    A node is a data structure representing a tile in a map
*/
typedef struct Node {
    status_t status;


} node_t;

/*
    Helper function to determine is a node is free
*/
extern int isfree(node_t *node);



