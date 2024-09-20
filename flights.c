#include "flights.h"


void chooseRandom(flightplan_t *plan, map_t *map, int id) {
    //choose random index for start, check if free, repeat for end
    srand((unsigned int)clock());

    int n = rand() % getSize(map);

    while(!isfree(pullIndex(map, n))) {
        n = rand() % getSize(map);
    }

    plan->start = n;
    n = rand() % getSize(map);

    while(!isfree(pullIndex(map, n)) || n == plan->start) {
        n = rand() % getSize(map);
    }

    plan->end = n;
    plan->id = id;

    //modify flightplan
}


int pathfind(flightplan_t *plan, map_t *map, double *compTime) {
    clock_t beginTime = clock();

    // pull plan into convenient format
    //int *start[2], *end[2];
    //getPosition(map, plan->start, start[0], start[1]);
    //getPosition(map, plan->end, end[0], end[1]);

    //start at start node

    node_t *nodeOfInterest = pullIndex(map, plan->start);
    node_t *startNode = nodeOfInterest;
    node_t *goalNode = pullIndex(map, plan->end);

    //allocate list of pointers to nodes size I might actually need
    calc_heuristic(map, startNode, goalNode); //just doing this to guess the length I'll need
    int guessListLength = startNode->hcost; //guess for how long of an array I need
    printf("guessing list length as %d \n", guessListLength);
    node_t **list = malloc(sizeof(node_t*)*guessListLength); //allocate my search list
        // this is a list of pointers to nodes which I will collect
    int listLength = 1; //set length to 1
    list[0] = startNode; //add the starter node
    startNode->gcost = 0; //this should be the case by default, but still
    startNode->seen = plan->id; //set the seen property to the id of this flight, its "been seen by this flight"
    displayNode(startNode);
    int failed = 0;
    

    while(nodeOfInterest != goalNode) { //until I find the node
        //usleep(100000);
        //first, let's find the node we want to look at
        if(listLength >= getSize(map)*(1-map->density)) {
            failed = 1;
            break;
        }

        //find the node with the lowest f cost in the list of search node
        //printf("Searching list for lowest f cost");
        int lowestF = __INT_MAX__;
            for (int i = 0; i < listLength; i++) { //loop through the list
                //loop through my search list
                //printf("On index %d, found ", i);

                node_t *currentNode = list[i];
                //displayNode(currentNode);
                if(fcost(currentNode) < lowestF) { //if this is the new lowest cost
                    nodeOfInterest = currentNode; //update the node of interest
                    lowestF = fcost(currentNode);
                }
                if(currentNode == goalNode) {
                    nodeOfInterest = currentNode;
                    continue;
                }
            }
            //printf("Found node with lowest cost: ");
            //displayNode(nodeOfInterest);

        // node of interest is now the node with the lowest total cost
        //find all the neighbors of the current node
        node_t *currentNode;
        //printf("Finding neighbors and adding to list \n");
        for(int dx = -1; dx <=1; dx++) { 
            for(int dy = -1; dy <=1; dy++) {
                int x = nodeOfInterest->x + dx;
                int y = nodeOfInterest->y + dy;
                int n;
                if(x < 0 || y < 0 || x >= map->l || y >= map->w) {
                    //printf("Neighbor failed validity check x: %d y: %d\n", x, y);
                    continue;
                }
                getIndex(map, x, y, &n);
                currentNode = pullIndex(map, n);
                if(!isfree(currentNode)) {
                    //printf("Neighbor failed free check x: %d y: %d\n", x, y);
                    continue;
                }
                    //check the node has a valid location
                    //ensure that it is free
                if(currentNode->seen == plan->id) { //if its already been seen
                    //printf("Neighbor has been seen:");
                    //displayNode(currentNode);
                    //no need to recalc h cost
                    //see if the g cost can be lowered
                    float newGCost = nodeOfInterest->gcost + ((dx != 0 && dy != 0) ? 2 : 1);
                    currentNode->gcost = (currentNode->gcost > newGCost) ? newGCost : currentNode->gcost; //replace the g cost if lower
                    currentNode->prev = (currentNode->gcost > newGCost) ? nodeOfInterest : currentNode->prev; //replace the prev node pointer if lower
                    //printf("Neighbor has been updated:");
                    //displayNode(currentNode);
                     
                        //if it can, recalc and repoint prev to be node of interest
                } else { //if it hasn't been seen
                    //printf("Neighbor has not been seen:");
                    //displayNode(currentNode);
                    calc_heuristic(map, currentNode, goalNode);   //calc h cost for it
                    currentNode->gcost = nodeOfInterest->gcost + ((dx != 0 && dy != 0) ? 2 : 1); //calc g cost for it
                    currentNode->prev = nodeOfInterest; //point it's prev to this node
                    currentNode->seen = plan->id;
                    
                    list[listLength++] = currentNode;
                    //printf("List Length: %d Neighbor has been updated:", listLength);
                    //displayNode(currentNode);
                    //add it to the list
                }

                //here's what else has to happen
                //need a clean exit

            }
        }

        //printf("new iteration \n");
            

        //what condition indicates that I failed?
    }

    free(list);

    if(failed) return -1; 

    //cycle back from goalNode to startNode by prev setting each one to occupied
    node_t *nodeOnReturn = goalNode;
    while(nodeOnReturn != startNode) {
        nodeOnReturn->status = OCCUPIED;
        nodeOnReturn = nodeOnReturn->prev;
    }

    //find neighbors and add to list
        //calculate g cost for each neighbor (only replace if lower)
        //calculate h cost of each neighbor (only if it hasn't been seen)
        //designate each neighbor as seen
        //designate the node of interest as the previous node's prev
        //calculate f cost for each node
        //select lowest f cost node and repeat
        //when we find the final node we're good

    //estimate the maximum number of items I'm gonna realistically need


    

    //A* my way to the end node

    //compile and record flight plan correctly

    //update map_t

    clock_t endTime = clock();
    *compTime = endTime - beginTime;
}

int fcost(node_t *node) {
    return node->gcost + node->hcost;
}

void printplan(flightplan_t *plan, map_t *map) {
    int xs, ys, xe, ye;
    getPosition(map, plan->start, &xs, &ys);
    getPosition(map, plan->end, &xe, &ye);

    printf("Flight from: (%d,%d) to (%d,%d) with id %d\n", xs, ys, xe, ye, plan->id);
}

/*
    Helper function to determine is a node is free
*/
int isfree(node_t *node) {
    return node->status == FREE;
}

void getPosition(map_t *map, int n, int *x, int *y) {
    *x = n % map->l ;
    *y = (n - *x) / map->l;
}


void getIndex(map_t *map, int x, int y, int *n) {
    *n = y*map->l + x;
}

void calc_heuristic(map_t *map, node_t *nodeOfInterest, node_t *goalNode) {
    int x1, x2, y1, y2;
    x1 = goalNode->x;
    x2 = nodeOfInterest->x;
    y1 = goalNode->y;
    y2 = nodeOfInterest->y;
    double hc_sq = pow(x1 - x2,2) + pow(y1 - y2,2);
    nodeOfInterest->hcost = (float) hc_sq;
}

/*
    Returns a pointer to a blank map with given length and width
*/
map_t *create_blank_map(unsigned int l, unsigned int w) {
    map_t* map = (map_t*) malloc(sizeof(map_t));
    map->l = l;
    map->w = w;
    map->grid = (node_t *) malloc(sizeof(node_t)*l*w);
    for(int i = 0; i < l*w; i++) {
        node_t *node = &(map->grid[i]);
        node->index = i;
        int x, y;
        getPosition(map, i, &x, &y);
        node->x = x;
        node->y = y;
        node->seen = 0;
        node->hcost = 0;
        node->gcost = 0;
        node->prev = NULL;
        node->status = FREE;
    }
    return map;
}

/*
    Populates a map with a number of obstacles
*/
void populate_with_obstacles(map_t* map, float density) { // change to integer function
    //TODO: add check for given flight paths
    srand((unsigned int)clock());

    // Check density is within a valid range (0-1)
    if(density >= 1 || density < 0) return; //remove this
    map->density = density;
    //Calculate the number of obstacles as the density times the map size
    int numObstacles = density*getSize(map);
    int lastIndex = -1;
    for(int i = 0; i < numObstacles; i++) { //for each obstacle to place

        int n;
        //choose an index to place the obstacle at

        if (lastIndex == -1) { //first round or lastIndex is cleared
            n = rand() % getSize(map);

        } else if(rand() % 2 == 0) { //hit the 50% chance to stick to last index
            int x, y;
            getPosition(map, lastIndex, &x, &y);
            x += ((rand() % 2) == 0) ? 1 : -1;
            y += ((rand() % 2) == 0) ? 1 : -1;
            n = x*map->w + y;
        } else { //do not hit the 50% chance to stick to last index
            n = rand() % getSize(map); //choose a random index
        }        
        
        if(n < 0 || n >= getSize(map) || pullIndex(map, n)->status == OBSTACLE) { //index must not meet these conditions
            //do not change lastIndex, try again
            i--; //need to repeat this iteration
            lastIndex = -1;
            //printf("%d \n", n);

        } else { //index is fine
            pullIndex(map, n)->status = OBSTACLE;
            lastIndex = n; //update lastindex
        } 
    }

}

/*
    Get a node at a specific index. Returns null if index invalid
*/
node_t *pullIndex(map_t* map, int n) {
    return &(map->grid[n]);
}

int getSize(map_t *map) {
    return (map->l)*(map->w);
}

void displayMap(map_t *map) {
    printf("l: %d w: %d size: %d \n", map->l, map->w, getSize(map));
    for(int i = 0; i < getSize(map); i++) {
        if(i % map->w == 0) { //end of line
            printf("\n");
        }
        if(map->grid[i].status == OCCUPIED) {
            printf("*");
        } else {
            printf("%d", map->grid[i].status);
        }      
    }
    printf("\n");
}

extern void displayNode(node_t * node) {
    printf("Node: n: %d x: %d y:%d seen: %d h: %f g: %f status: %d\n", node->index, node->x, node->y, node->seen, node->hcost, node->gcost, node->status);
}


extern void freeGrid(map_t *map) {
    free(map->grid);
}

