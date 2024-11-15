#include "pathfind.h"
#include <unistd.h>


const float wx = 1;
const float wy = 1;
const float wz = 2;
const float wpsi = 0.1;
const float wv = 1;
const float wt = 1;

/*
    Known issues:
        Something's up with the weighting that makes things cycle sometimesz
        Things are probably being re-added to the open list for some reason
        The open and closed lists should really be replaced with hash tables or min heaps

        Update "wait a step to launch" manuever to be as expensive as the entire heuristic

        Need to actually implement spaceclaim and spacecheck

*/

heap_t* kinematicNeighbors(state_t* state, vehicle_t vehicle) {
    heap_t *h = createHeap(8); //8 maximum possible neighbors
    //cruise
    state_t *cruise = (state_t*)calloc(1,sizeof(state_t));
    cruise->position.x = state->position.x + cos(state->heading);
    cruise->position.y = state->position.y + sin(state->heading);
    cruise->position.z = state->position.z;
    cruise->time = state->time + 1/state->velocity;
    cruise->velocity = state->velocity;
    cruise->heading = state->heading;
    cruise->previous = state;
    cruise->maneuver = CRUISE;
    insert(h, cruise);
    //accelerate
    state_t *accelerate = (state_t*)calloc(1,sizeof(state_t));
    accelerate->position.x = state->position.x + cos(state->heading);
    accelerate->position.y = state->position.y + sin(state->heading);
    accelerate->position.z = state->position.z;
    accelerate->velocity = sqrt((state->velocity)*(state->velocity) + 2*vehicle.vdot);
    accelerate->time = state->time + 2/(state->velocity + accelerate->velocity);
    accelerate->heading = state->heading;
    accelerate->previous = state;
    accelerate->maneuver = ACCELERATE;
    insert(h, accelerate);
    //decelerate
    state_t *decelerate = (state_t*)calloc(1,sizeof(state_t));
    decelerate->position.x = state->position.x + cos(state->heading);
    decelerate->position.y = state->position.y + sin(state->heading);
    decelerate->position.z = state->position.z;
    decelerate->velocity = sqrt((state->velocity)*(state->velocity) - 2*vehicle.vdot);
    decelerate->time = state->time + 2/(state->velocity + decelerate->velocity);
    decelerate->heading = state->heading;
    decelerate->previous = state;
    decelerate->maneuver = DECELERATE;
    //check stall speed
    if(decelerate->velocity > vehicle.vstall) {
        insert(h, decelerate);
    } else {
        free(decelerate);
    }
    //turn
    //left
    state_t *left = (state_t*)calloc(1,sizeof(state_t));
    left->position.x = state->position.x + cos(state->heading - 0.5*vehicle.psidot*(1/state->velocity));
    left->position.y = state->position.y + sin(state->heading - 0.5*vehicle.psidot*(1/state->velocity));
    left->position.z = state->position.z;
    left->velocity = state->velocity;
    left->time = state->time + 1/state->velocity;
    left->heading = fmod(state->heading - vehicle.psidot*(1/state->velocity),2*M_PI);
    left->previous = state;
    left->maneuver = LEFT;
    insert(h, left);
    //right
    state_t *right = (state_t*)calloc(1,sizeof(state_t));
    right->position.x = state->position.x + cos(state->heading - 0.5*vehicle.psidot*(1/state->velocity));
    right->position.y = state->position.y + sin(state->heading - 0.5*vehicle.psidot*(1/state->velocity));
    right->position.z = state->position.z;
    right->velocity = state->velocity;
    right->time = state->time + 1/state->velocity;
    right->heading = fmod(state->heading - vehicle.psidot*(1/state->velocity),2*M_PI);
    right->previous = state;
    right->maneuver = RIGHT;
    insert(h, right);
    //climb not done
    state_t *climb = (state_t*)calloc(1,sizeof(state_t));
    climb->position.x = state->position.x + cos(state->heading);
    climb->position.y = state->position.y + sin(state->heading);
    climb->position.z = state->position.z + vehicle.hdot/state->velocity;
    climb->velocity = state->velocity;
    climb->time = state->time + 1/(state->velocity);
    climb->heading = state->heading;
    climb->previous = state;
    climb->maneuver = CLIMB;
    insert(h, climb);
    //descend not done
    state_t *descend = (state_t*)calloc(1,sizeof(state_t));
    descend->position.x = state->position.x + cos(state->heading);
    descend->position.y = state->position.y + sin(state->heading);
    descend->position.z = state->position.z - vehicle.hdot/state->velocity;
    descend->velocity = state->velocity;
    descend->time = state->time + 1/(state->velocity);
    descend->heading = state->heading;
    descend->previous = state;
    descend->maneuver = DESCEND;
    insert(h, descend);
    //need the delay maneuver
    //wait ONLY if I am directly at the start position
    if(state->maneuver == YETTOMOVE) {
        state_t *delay = (state_t*)calloc(1,sizeof(state_t));
        delay->position.x = state->position.x;
        delay->position.y = state->position.y;
        delay->position.z = state->position.z;
        delay->velocity = state->velocity;
        delay->time = state->time + 1;
        delay->heading = state->heading;
        delay->previous = state;
        delay->maneuver = YETTOMOVE; 
        insert(h, delay);
    }
    return h;
}

int boundsCheck(map_t *map, state_t *state) {
    return (state->position.z <= map->H) && (state->position.z >= 0) && 
           (state->position.x <= map->W) && (state->position.x >= 0) && 
           (state->position.y <= map->L) && (state->position.y >= 0);
}

int spaceCheck(map_t *map, state_t *state, vehicle_t vehicle){
    //find where I would actually be at this point in time
    int tLower = floor(state->time);
    int tHigher = ceil(state->time);
    if(tLower - tHigher < 0.05) { //avoid some floating point errors
        // I'm on "integer time"
        int xcoord = floor(state->position.x); //acquire the coordinate
        int ycoord = floor(state->position.y); 
        int zcoord = floor(state->position.z);
        int val = pull(map, xcoord, ycoord, zcoord, tLower);
        if(val == 0) {
            //free node! 
            return 1;
        } else if (val == 1) {
            return 0;
        } else if (val == 2) {
            //I need to do a detailed check
            return 1; //REPLACE
        }
    } else { //non-integer time
        int valLower = pull(map, node->point.x, node->point.y, node->point.z, tLower);
        int valHigher = pull(map, node->point.x, node->point.y, node->point.z, tHigher);
        if(valLower == 0 && valHigher == 0) {
            return 1;
        } else if (valLower == 1 || valHigher == 1) {
            return 0;
        } else if (!(valLower == 2) !=  !(valHigher == 2)) { //one is a 2, one is a 0
            //find which one is the two
            return 1; //REPLACE
        } else { //they're both 2s
            return 1; //REPLACE
        }
        // I'm not on "integer time"
    }
}

void spaceClaim(map_t *map, state_t *state);

int closeEnough(map_t *map, state_t* a, point_t b) {
    //purely a position thing here
    //check if two states are close enough in position to be considered identical
    return (pow((a->position.x) - b.x,2) + pow((a->position.y) - b.y,2) + pow((a->position.z) - b.z,2)) < 1 ? 1 : 0;
}

void calcCosts(state_t *state, vehicle_t vehicle, point_t endPoint) {
    state_t *prev = state->previous;
    float dx = state->position.x - prev->position.x;
    float dy = state->position.y - prev->position.y;
    float dz = state->position.z - prev->position.z;
    float dpsi = fmod(state->heading - prev->heading, 2*M_PI);
    float dv = state->velocity - prev->velocity;
    float dt = state->time - prev->time;
    float dg = sqrt(wx*pow(dx,2) + wy*pow(dy,2) + wz*pow(dx,2) + wpsi*pow(dpsi,2) + wv*pow(dv,2) + wt*pow(dt, 2));
    state->g = prev->g + dg;
    state->h = dist(state->position, endPoint);
}

state_t* initialState(point_t startPoint, point_t endPoint, int t0, vehicle_t vehicle) {
    //create an allocate state object
    //initial velocity is preferred velocity?
    //t is the initial tau

    state_t* state = (state_t*)malloc(sizeof(state_t));
    state->position = startPoint;
    state->time = t0;
    state->maneuver = YETTOMOVE;
    state->previous = NULL;
    state->velocity = vehicle.vstall;
    state->heading = atan2(endPoint.y - startPoint.y, endPoint.x - startPoint.x);
    state->g = 0;
    state->h = dist(state->position, endPoint);
    return state;
}

heap_t* trimNeighbors(map_t *map, heap_t *old, vehicle_t vehicle) {
    heap_t *new = createHeap(8); //8 maximum
    for(int i = 0; i < old->size; i++) { //for each neighbor
        state_t *state = extract(old);
        if (!boundCheck(map, state)) { //if I fails the easy bounds check
            //invalid node
            free(state); //forget it
        } else if (spaceCheck(map, state, vehicle)) { // I pass the space check
            insert(new, state);
        } else { //failed the space check
            free(state);
        }
    }
    // for each neighbor
        //check if it's out of bounds
        //check if it's dangerous
            //if we reject one, free it and decrement the count

}

void addPathToTensor(ot_t *ot, path_t* path) {
    //add a solved path to a tensor - this is the geonfencing part
    int tf0 = ot->tf;
    int tf1 = ceil(path->tf);
    printf("tf0: %d, tf1: %d", tf0, tf1);
    if(tf1 > tf0) { //my new path demands an extended time horizon
        //extend the time horizon of the tensor
        extendTimeHorizon(ot, ceil(tf1-tf0));
    }
    //populate the tensor with the path's waypoints geofenced
    for(int i = 0; i < path->pathLength; i++) {
        wp_t wp = path->waypoints[i];
        printf("Waypoint: (%d,%d,%d) at t = %f setting values at t = %d and t = %d to 1", wp.location.x, wp.location.y, wp.location.z, wp.time, (int) floor(wp.time), (int) ceil(wp.time));
        addWaypointToTensor(ot, wp);
        //printf("Setting a value \n");
    }
}

void pathFind(map_t *map, path_t *path) {
    /*
    psuedocode:
        start the timer

        grab the current occupancy map

        create the initial state
        calculate its costs

        create openlist minheap
        add initial node
        create closedlist hashtable

        while placed remain to be explored
            pull the lowest cost node

            check if its the goal or close enough to the goal


        find all the kinematically feasible neighboring nodes

        trim the neighbors by bounds

        spacecheck each node for occupational feasibility

        add winners to the open list
    
    */
    clock_t beginTime = clock();

    heap_t *openList = createHeap(9999); //uhhhhhh
    int success = 0;

    state_t* nodeOfInterest = initialState(path->startPoint, path->endPoint, path->ideal_tau_start, path->vehicle);
    insert(openList, nodeOfInterest);

    while(openList->size > 0) {
        nodeOfInterest = extract(openList); //pop best node off minheap

        if (closeEnough(map, nodeOfInterest, path->endPoint)){
            success = 1;
            break; //we're there
        } 

        //create the list of kinematically feasible neighbors
        heap_t *kN = kinematicNeighbors(nodeOfInterest, path->vehicle);
        heap_t *N = trimNeighbors(map, kN, path->vehicle);
        free(kN);

        //trim the list for geometric & occpancy reasons

        for(int i = 0; i < N->size; i++) {
            state_t *neighbor = extract(N);
            calcCosts(neighbor, path->vehicle);
            insert(openList, neighbor);
        }
        free(N);
    }

    //path solved, or not lol







    freeHeap(openList);
    return

    //start the timer

    //printf("Finding Path from (%d,%d,%d) -> (%d,%d,%d) \n", start.x, start.y, start.z, end.x, end.y, end.z);
    //grab the tensor from the current map
    ot_t* ot = map->occupancytensor;
    int tf = ot->tf;

    //create the open list and add the start node to it
    node_t **openList = (node_t**) calloc(1, sizeof(node_t*));
    node_t **closedList = (node_t**) calloc(1, sizeof(node_t*));

    int openCount = 1;
    int closedCount = 0;
    int success = 0;

    node_t* nodeOfInterest;
    openList[0] = makeStarterNode(start, end);
    //calculate its heuristic

    //create the closed list

    while(openCount > 0) { //while open list remains not empty
        //find the node on the open list with the lowest f cost
        float lowestF = 9999;
        int index = -1;
        for(int i = 0; i < openCount; i++) {
            node_t *currentNode = openList[i];
            if(fcost(currentNode) < lowestF) {
                lowestF = fcost(currentNode);
                index = i;
                nodeOfInterest = currentNode;
            }
        }
        //printf(nodeOfInterest == NULL);
        printf("Node of Interest: (%d,%d,%d) @ %f w/ g = %f, h = %f, f = %f \n", nodeOfInterest->point.x, nodeOfInterest->point.y, nodeOfInterest->point.z, nodeOfInterest->t, nodeOfInterest->gcost, nodeOfInterest->hcost, fcost(nodeOfInterest));
            //check if that node is the goal
                //if yes, win
        if(match(nodeOfInterest->point, end)) {
            success = 1;
            break;
        }

        removeFromList(&openList, &openCount, index);
        addToList(&closedList, &closedCount, nodeOfInterest);
            
        int numNeighbors = 0;
        node_t **neighborList = getNeighbors(map, nodeOfInterest, end, &numNeighbors);
        
        for(int i = 0; i < numNeighbors; i++) {
            node_t *neighbor = neighborList[i];
            int alreadySeen = 0;
            for(int j = 0; j < closedCount; j++) {
                if(match(neighbor->point, closedList[j]->point) && abs(closedList[j]->t - neighbor->t) < 0.1) {
                    alreadySeen = 1;
                    break;
                }
            }
            for(int j = 0; j < openCount; j++) {
                if(match(neighbor->point, openList[j]->point) && abs(openList[j]->t - neighbor->t) < 0.1) {
                    alreadySeen = 1;
                    break;
                }
            }
            if(alreadySeen == 1) {
                //printf("Triggered already seen condition");
                free(neighbor);
            } else if(neighbor->t > (tf-1)) { //I'm past the current time horizon
                addToList(&openList, &openCount, neighbor); //add to the open list
            } else if(spaceCheck(map, neighbor)) { //I pass the space check
                addToList(&openList, &openCount, neighbor);
            } else { //I can't add this node for some reason
                free(neighbor);
            }
        }
        free(neighborList);
        //printf("Open list Size: %d, Closed List Size: %d\n", openCount, closedCount);
        /*
        for(int i = 0; i < openCount; i++) {
            node_t *node = openList[i];
            printf("    Open list node: (%d,%d,%d) @ %f w/ g = %f, h = %f, f = %f \n", node->point.x, node->point.y, node->point.z, node->t, node->gcost, node->hcost, fcost(node));

        }
        for(int i = 0; i < closedCount; i++) {
            
        }
        usleep(100000);
        */
        
    }

    path_t path;
    path.solved = success;
    //planner is done
    if(success == 0) {
        //backtrack and create path object (this is a string of waypoints for a vehicle)
    } else { 
        node_t *nodeOnReturn = nodeOfInterest;
        int pathLength = 0;
        while(nodeOnReturn != NULL) {
            pathLength++;
            nodeOnReturn = nodeOnReturn->prev;
        }
        path.pathLength = pathLength;
        path.waypoints = (wp_t*) calloc(pathLength, sizeof(wp_t));
        int i = 0;
        path.tf = nodeOfInterest->t;
        do {
            //printf("Node: %d,%d,%d,%f", nodeOfInterest->point.x,nodeOfInterest->point.y,nodeOfInterest->point.z,nodeOfInterest->t);
            path.waypoints[pathLength - i -1] = nodeToWaypoint(nodeOfInterest);
            nodeOfInterest = nodeOfInterest->prev;
            i++;
        } while(nodeOfInterest != NULL);
 
    }
    clock_t endTime = clock();
    path.timeToSolvems =  1000*((double)(endTime - beginTime))/CLOCKS_PER_SEC;
    
    freeList(openList, openCount);
    freeList(closedList, closedCount);
    free(openList);
    free(closedList);
    
    return path;
}
  
int spaceCheck(map_t *map, node_t *node) {
    //check if I have space to place a geofence here


    //if I have all zeros, I'm good

    //if I have a 1, I'm not good


    //if I have a 2
        //collect every place where I would place a 1 if I were to be here
        //if any of them are already 1s, im cooked
    
}

void printPath(path_t path);

heap_t* createHeap(int capacity) {
    heap_t *h = (heap_t*)malloc(sizeof(heap_t));

    h->size = 0;
    h->capacity = capacity;

    h->arr = (state_t**)malloc(capacity*sizeof(state_t*));

}

void insert(heap_t *h, state_t* value) {
    h->arr[h->size] = value; //insert new value at end

    int i = h->size;
    h->size++;

    //bubble up
    while (i != 0 && f_value(h->arr[i]) < f_value(h->arr[(i - 1) / 2])) {
        swap(&h->arr[i], &h->arr[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

state_t* extract(heap_t *h) {
    state_t *min = h->arr[0];
    h->arr[0] = h->arr[h->size - 1];
    h->size--;

    // Bubble down
    int i = 0;
    while (2 * i + 1 < h->size) {
        int smallest = i;
        int left = 2 * i + 1;
        int right = 2 * i + 2;

        if (left < h->size && f_value(h->arr[left]) < f_value(h->arr[smallest])) {
            smallest = left;
        }
        if (right < h->size && f_value(h->arr[right]) < f_value(h->arr[smallest])) {
            smallest = right;
        }
        if (smallest == i) break;

        swap(&h->arr[i], &h->arr[smallest]);
        i = smallest;
    }

    return min;
}

void printHeap(heap_t *h);

void swap(state_t **a, state_t **b) {
    state_t *temp = *a;
    *a = *b;
    *b = temp;
}

float f_value(state_t* state) {
    return state->g + state->h;
}

extern void freeHeap(heap_t *h) {
    free(h->arr);
    free(h);
}