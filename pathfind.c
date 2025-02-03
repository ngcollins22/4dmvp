#include "pathfind.h"
#include <unistd.h>


const float CHI = 15; //meters
const float TAU = 1; //seconds
const float G_CANONCIAL = 9.81 * CHI / (TAU*TAU); // Canonical g value

/*
    Source file containing functions and utilities for pathfinding
*/

const float w = 0.5; // Weight parameter of g-cost function.
const float GREEDY_MULTIPLIER = 1; // Multiplier applied to heuristc
/*
    Affects the algorithm somewhat. Suggest increasing this number if solutions become too slow.
    Encourages algorithm to explore in direction of decreasing heuristic faster than low path-costs.
    Reduces the typical optimality of found solution in exchange for increasing solution time.
*/


// ID counter used to distinguish paths. Mostly used for exporting data for presentation.
int global_id_counter = 0;


// Set of tolerances for the following closeEnoughState function. 
// Recommend setting these below or at the corresponding parameters of the vehicle.
const float vTol = 0.05;
const float psiTol = 0.1;
const float ttol = 0.5;

heap_t* kinematicNeighbors(state_t* state, vehicle_t vehicle) {
    heap_t *h = createHeap(8); //8 maximum possible neighbors (8 distinct maneuvers)
    // A heap is used here for convenience
    // This function performs NO geofencing logic.
    /*
        These maneuvers use the kinematics described in the slides.
    */
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
    cruise->isFinalSolution = 0;
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
    accelerate->isFinalSolution = 0;
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
    decelerate->isFinalSolution = 0;
    if(decelerate->velocity > vehicle.vstall) { //Perform minimum-speed check.
        insert(h, decelerate);
    } else {
        free(decelerate);
    }
    //left turn
    state_t *left = (state_t*)calloc(1,sizeof(state_t));
    left->position.x = state->position.x + cos(state->heading - 0.5*vehicle.psidot*(1/state->velocity));
    left->position.y = state->position.y + sin(state->heading - 0.5*vehicle.psidot*(1/state->velocity));
    left->position.z = state->position.z;
    left->velocity = state->velocity;
    left->time = state->time + 1/state->velocity;
    left->heading = fmod(state->heading - vehicle.psidot*(1/state->velocity),2*M_PI);
    left->previous = state;
    left->maneuver = LEFT;
    left->isFinalSolution = 0;
    insert(h, left);
    //right turn
    state_t *right = (state_t*)calloc(1,sizeof(state_t));
    right->position.x = state->position.x + cos(state->heading + 0.5*vehicle.psidot*(1/state->velocity));
    right->position.y = state->position.y + sin(state->heading + 0.5*vehicle.psidot*(1/state->velocity));
    right->position.z = state->position.z;
    right->velocity = state->velocity;
    right->time = state->time + 1/state->velocity;
    right->heading = fmod(state->heading + vehicle.psidot*(1/state->velocity),2*M_PI);
    right->previous = state;
    right->maneuver = RIGHT;
    right->isFinalSolution = 0;
    insert(h, right);
    //climb
    state_t *climb = (state_t*)calloc(1,sizeof(state_t));
    climb->position.x = state->position.x + cos(state->heading);
    climb->position.y = state->position.y + sin(state->heading);
    climb->position.z = state->position.z + vehicle.hdot/state->velocity;
    climb->velocity = state->velocity;
    climb->time = state->time + 1/(state->velocity);
    climb->heading = state->heading;
    climb->previous = state;
    climb->maneuver = CLIMB;
    climb->isFinalSolution = 0;
    insert(h, climb);
    //descend
    state_t *descend = (state_t*)calloc(1,sizeof(state_t));
    descend->position.x = state->position.x + cos(state->heading);
    descend->position.y = state->position.y + sin(state->heading);
    descend->position.z = state->position.z - vehicle.hdot/state->velocity;
    descend->velocity = state->velocity;
    descend->time = state->time + 1/(state->velocity);
    descend->heading = state->heading;
    descend->previous = state;
    descend->maneuver = DESCEND;
    descend->isFinalSolution = 0;
    insert(h, descend);
    //Delay maneuver
    if(state->maneuver == YETTOMOVE) { //This maneuver is only permitted if the vehicle hasn't taken off yet
        state_t *delay = (state_t*)calloc(1,sizeof(state_t));
        delay->position.x = state->position.x;
        delay->position.y = state->position.y;
        delay->position.z = state->position.z;
        delay->velocity = state->velocity;
        delay->time = state->time + 1;
        delay->heading = state->heading;
        delay->previous = state;
        delay->maneuver = YETTOMOVE; 
        delay->isFinalSolution = 0;
        insert(h, delay);
    }
    return h;
}

int boundsCheck(map_t *map, state_t *state) {
    //Simply checks the state's position against the bounds of the map
    return (state->position.z <= map->H) && (state->position.z >= 0) && 
           (state->position.x <= map->W) && (state->position.x >= 0) && 
           (state->position.y <= map->L) && (state->position.y >= 0);
}

int spaceCheck(map_t *map, state_t *state, vehicle_t vehicle){
    /*
        Check the tensor for occupancy from the previous state x,y,z,t to the new state x,y,z,t
        Determines if the state is feasible. Called relatively infrequently.
        Should not ever get called on the first node.
    */ 
    point_t p1 = state->previous->position; //pull previous state's position
    float t1 = state->previous->time; //pull previous state's time
    point_t p2 = state->position; //pull this state's position
    float t2 = state->time; //pull this state's time
    /*
        This routine steps (in units of 1 chi) from the previous state to the current state,
        intentionally undershooting, and then checks the final position. 
        This approach ensures every spatial integer location is checked between both states.
        At each spatial location, all adjacent temporal locations are checked as well. This
        enforces both a spatial and temporal geofence.
    */
    float length = dist(state->position, state->previous->position); //total length of traverse
    float nx = (p2.x - p1.x)/length; //normal vector x component
    float ny = (p2.y - p1.y)/length; //normal vector y component
    float nz = (p2.z - p1.z)/length; //normal vecotr z component
    int lengthInt = ceil(length); //integer ceiling of length
    //this loop can "integer miss" the final point, so we still need to manually check the final location
    //cannot attempt to overshoot because it may exceed bounds!
    for(int i = 0; i < lengthInt; i++) { //i is the step, of unit length, along the path
        point_t controlPoint = add(p1, nx*i, ny*i, nz*i); //find float coordinate of this control point
        int xc = (int)floor(controlPoint.x); //find int coordinates of the control point
        int yc = (int)floor(controlPoint.y);
        int zc = (int)floor(controlPoint.z);
        //Integer times to check are both integer sides of previous and final state times
        int times[4] = {(int)floor(t1), (int)ceil(t1), (int)floor(t2), (int)ceil(t2)};
        for(int j = 0; j < 4; j++) { //At this control point, check each temporal location
            if(times[j] >= map->occupancytensor->tf) continue; //if the time is beyond the final time of the map, it's not occupied
            int val = pull(map, xc, yc, zc, times[j]); //query value of the map here
            if(val == 0) { //This voxel is free
                continue; // move onto the next time index to check
            } else if (val == 1) { //This voxel is within a geofence -> NOT safe
                return 0; //a single non-free means it is not safe to traverse here
                // return zero to tell algorithm this traverse is unsafe
            }  else if (val == 2) { //This voxel is close to a geofence -> perform detailed check
                // essentially this part casts the sphere that the vehicle would place here if it traveled through here
                // It then checks if the geofence it would place here overlaps with another geofence
                // If it determines any overlap, this traverse is declared unsafe.
                for(int dx = -vehicle.R; dx <= vehicle.R; dx++) { //classic triple loop around this position at radius of vehicle geofence
                    for(int dy = -vehicle.R; dy <= vehicle.R; dy++) {
                        for(int dz = -vehicle.R; dz <= vehicle.R; dz++) {
                            if(dx == 0 && dy == 0 && dz == 0) continue; //skip this exact point
                            if(sqrt(pow(dx,2) + pow(dy,2) + pow(dz,2)) > vehicle.R) continue; //check if this poitions is actually within the sphere (loops produce a cube)
                            if(xc + dx >= map->W || xc + dx < 0 ||
                               yc + dy >= map->L || yc + dy < 0 ||
                               zc + dz >= map->H || zc + dz < 0) continue; //check if this location is out of bounds of the map
                            int neighborVal = pull(map, xc + dx, yc + dy, zc + dz, times[j]); //pull the voxel value at these coordinates.
                            if(neighborVal == 1) return 0; //if a 1 is found, this traverse is not allowed
                        }
                    }
                }
            }
        }

    }
    return 1; //if all check are passed, this is a safe traverse. Return 1.
}

float dist(point_t p1, point_t p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)); //cartesian distance formula
}

point_t add(point_t p, float dx, float dy, float dz) {
    p.x+=dx; //simply copies and returns a point with offsets added
    p.y+=dy;
    p.z+=dz;
    return p;
}

void writePath(map_t *map, path_t *path) {
    //Iteratively follows the path solution linked list and updates the map with it
    state_t *currentNode = path->solution; //Root pointer of the path solution, and also the final state of the trajectory
    int ntf = (int)ceil(path->solution->time); //Find the final time of the path
    //extend the time horizon of the map to accomodate the time of the path.
    if(ntf > map->occupancytensor->tf) extendTimeHorizon(map->occupancytensor, ntf - (map->occupancytensor->tf)); //extend the time horizon to account for the new path
    while(currentNode->previous != NULL) { //iteratively add each state to the map
        spaceClaim(map, currentNode, path->vehicle); //calls complex routine that considers the traversal from the previous state to this one
        currentNode = currentNode->previous;
    }
}

void spaceClaim(map_t *map, state_t *state, vehicle_t vehicle) {
    /*
        This routine steps (in units of 1 chi) from the previous state to the current state,
        intentionally undershooting, and then checks the final position. 
        This approach ensures every spatial integer location is checked between both states.
        At each spatial and temporal location, a geofence and a keep out zone is placed into the map.
    */
    point_t p1 = state->previous->position;
    float t1 = state->previous->time;
    point_t p2 = state->position;
    float t2 = state->time;
    /*
        This routine steps (in units of 1 chi) from the previous state to the current state,
        intentionally undershooting, and then checks the final position. 
        This approach ensures every spatial integer location is checked between both states.
        At each spatial location, all adjacent temporal locations are checked as well. This
        enforces both a spatial and temporal geofence.
    */
    float length = dist(state->position, state->previous->position);
    float nx = (p2.x - p1.x)/length;
    float ny = (p2.y - p1.y)/length;
    float nz = (p2.z - p1.z)/length;
    int lengthInt = ceil(length);
    //this loop can "integer miss" the final point, so we still need to manually check the final location
    //cannot attempt to overshoot because it may exceed bounds!
    for(int i = 0; i < lengthInt; i++) { 
        point_t controlPoint = add(p1, nx*i, ny*i, nz*i); //control point step
        int xc = (int)floor(controlPoint.x); //find coordinates
        int yc = (int)floor(controlPoint.y);
        int zc = (int)floor(controlPoint.z);
        int times[4] = {(int)floor(t1), (int)ceil(t1), (int)floor(t2), (int)ceil(t2)};
        for(int j = 0; j < 4; j++) { //check each time value
            for(int dx = -2*vehicle.R; dx <= 2*vehicle.R; dx++) { //place 2s in the sphere of twice the radius of the vehicle geofence
                for(int dy = -2*vehicle.R; dy <= 2*vehicle.R; dy++) {
                    for(int dz = -2*vehicle.R; dz <= 2*vehicle.R; dz++) {
                        if(xc + dx >= map->W || xc + dx < 0 || //reject if the position is out of bounds
                            yc + dy >= map->L || yc + dy < 0 ||
                            zc + dz >= map->H || zc + dz < 0) continue;
                        if(sqrt(pow(dx,2) + pow(dy,2) + pow(dz,2)) > 2*vehicle.R) continue; //check if the position is actually within the double geofence radius
                        push(map, xc + dx, yc + dy, zc + dz, times[j], 0b10); //place a two
                    }
                }
            }
            for(int dx = -vehicle.R; dx <= vehicle.R; dx++) { //place 1s in a sphere with radius of the geofence of the vehicle
                for(int dy = -vehicle.R; dy <= vehicle.R; dy++) {
                    for(int dz = -vehicle.R; dz <= vehicle.R; dz++) {
                        if(xc + dx >= map->W || xc + dx < 0 || //reject if the position is out of bounds
                            yc + dy >= map->L || yc + dy < 0 ||
                            zc + dz >= map->H || zc + dz < 0) continue;
                        if(sqrt(pow(dx,2) + pow(dy,2) + pow(dz,2)) > vehicle.R) continue; //check if the position is actually within the geofence radius
                        push(map, xc + dx, yc + dy, zc + dz, times[j], 0b01); //place a 1 - will overwrite the twos placed before
                    }
                }
            }
        }
    }
}

int closeEnough(state_t* a, point_t b) {
    //Purely a 3D position check.
    //Checks if the distance between a state's position and the point is less than one.
    return (pow((a->position.x) - b.x,2) + pow((a->position.y) - b.y,2) + pow((a->position.z) - b.z,2)) < 1 ? 1 : 0;
}

int closeEnoughState(state_t* a, state_t* b) {
    return closeEnough(a, b->position) && //check if the positions are close enough
           fabs(a->velocity - b->velocity) < vTol &&  //check if velocities are close enough
           fabs(a->heading - b->heading) < psiTol &&  //check if headings are close enough
           fabs(a->time - b->time) < ttol; //check if times are close enough
}

void calcCosts(state_t *state, point_t endPoint, vehicle_t vehicle) {
    state_t *prev = state->previous;
    // Calculate Path Cost
    // First term is default power required to fly
    float path_d = sqrt(pow(state->position.x - prev->position.x,2) + pow(state->position.x - prev->position.x,2) + pow(state->position.z - prev->position.z,2));
    float deltaHbase = vehicle.W/vehicle.LD * path_d; //base energy cost
    if (state->maneuver == CRUISE) { //no maneuver
        state->g = prev->g + deltaHbase; //nothing interesting
    } else if (state->maneuver == ACCELERATE || state->maneuver == DECELERATE) { //Speed change type
        float vehicle_mass = vehicle.W/G_CANONCIAL; //vehicle mass in canonical units
        float deltaV = state->velocity - prev->velocity; //change in velocity
        float deltaT = state->time - prev->time; //change in time
        float a = deltaV/deltaT; //acceleration
        float F = vehicle_mass*a; //force
        float speedChangeWork = F * path_d; //work done to change speed
        state->g = prev->g + deltaHbase + speedChangeWork; //total energy cost
    } else if (state->maneuver == LEFT || state->maneuver == RIGHT) { //Turn type   
        float deltaPsi = fmod(abs(state->heading - prev->heading),2*M_PI); //change in heading
        float omega = deltaPsi/(state->time - prev->time); //angular velocity
        float bank_angle = atan(omega*state->velocity/G_CANONCIAL); //bank angle
        float load_factor = 1/cos(bank_angle); //load factor
        state->g = prev->g + deltaHbase*load_factor; //crude assumption that power required scales with load factor
    } else if (state->maneuver == CLIMB || state->maneuver == DESCEND) { //Climb type
        float deltah = state->position.z - prev->position.z; //change in altitude
        state->g = prev->g + deltaHbase + vehicle.W*deltah; //total energy cost
    } else if (state->maneuver == YETTOMOVE) { //Hover
        float deltaT = state->time - prev->time; //change in time
        state->g = prev->g + vehicle.hoverPower*deltaT; //total energy cost
    }

    // Calculate Heuristic
    float d = sqrt(pow(state->position.x - endPoint.x,2) + pow(state->position.y - endPoint.y,2) + pow(state->position.z - endPoint.z,2));
    float h = GREEDY_MULTIPLIER*(vehicle.W/vehicle.LD)*d; //heuristic formula, units of energy

    state->h = h; //heuristic formula from slides
}

state_t* initialState(point_t startPoint, point_t endPoint, int t0, vehicle_t vehicle) {
    //Special function to create the starter node of the algorithm
    state_t* state = (state_t*)calloc(1, sizeof(state_t));
    state->position = startPoint; //starting position
    state->time = t0;
    state->maneuver = YETTOMOVE; //maneuver representing vehicle hasn't taken off yet
    state->previous = NULL; //no previous state
    //Vehicle state initial conditions:
    state->velocity = vehicle.vpref; //vehicle preferred velocity
    state->heading = atan2(endPoint.y - startPoint.y, endPoint.x - startPoint.x); //vehicles start pointing towards their goal
    state->g = 0; //intial path cost is zero
    float d = sqrt(pow(state->position.x - endPoint.x,2) + pow(state->position.y - endPoint.y,2) + pow(state->position.z - endPoint.z,2));
    state->h = GREEDY_MULTIPLIER*(vehicle.W/vehicle.LD)*d; //heuristic formula, units of energy
    state->isFinalSolution = 0; //although this node will actually be a part of the final solution, flag is not set yet in case of failure.
    return state;
}

heap_t* trimNeighbors(map_t *map, heap_t *old, vehicle_t vehicle) {
    heap_t *new = createHeap(8); //8 neighbors maximum, if every state in the list passes these checks
    while(old->size > 0) { //for each neighbor
        state_t *state = extract(old); //extract the potential neighbor
        if (!boundsCheck(map, state)) { //check if state is out of bounds
            free(state); //if the state is out of bounds, free it.
        } else if (spaceCheck(map, state, vehicle)) { // check if the state conflicts with any existing geofences
            insert(new, state); //if it passes the spacecheck, it's a valid neighbor
        } else { //reaching this point means the spacecheck was failed
            free(state); //the state conflicts with an existing geofence. Free it.
        }
    }
    return new; //return a heap of all the neighbors that passed the checks.
}

void pathFind(map_t *map, path_t *path) {
    /*
        This is the main pathfinding loop. As described in the slides, this is fundamentally A* based.
        Open and closed lists are kept, and the major logic resembles an A* algorithm.
    */
    clock_t beginTime = clock(); // start the clock
    int success = 0; // int to check if the algorithm found a solution or not

    //initialize the search using the initial state
    state_t* nodeOfInterest = initialState(path->startPoint, path->endPoint, path->ideal_tau_start, path->vehicle);
    path->initialh = nodeOfInterest->h; //record the intial heuristc for stats
    
    //Allocate the open and closed lists. There is probably a better way to figure out maximum sizes, but this works.
    //I would recommend a numerical analysis based on each state has 8 neighbors max, and the distance from start to end.
    //Overallocating space is somewhat memory intensive, but is orders of magnitude faster than dynamic resizing.
    heap_t *openList = createHeap(10000000); // 10 million is plenty. Around 20MB.
    state_t **closedList = (state_t**) calloc(10000000, sizeof(state_t*));
    int closedCount = 0; //track the size of the closed list.
    insert(openList, nodeOfInterest); //insert the starter node into the list.

    while(openList->size > 0) { // While there are nodes remaining to be explored
        //Openlist is presorted with lowest cost node at the top
        nodeOfInterest = extract(openList); //Pop the lowest f-cost node off the open list
        
        //Debugging
        printf("        State: ");
        printPoint(nodeOfInterest->position);
        printf(" t: %.2f, v: %.2f, psi: %.2f, h: %.2f, g: %.2f, maneuver: %s\n", nodeOfInterest->time, nodeOfInterest->velocity, nodeOfInterest->heading, nodeOfInterest->h, nodeOfInterest->g, maneuver_to_string(nodeOfInterest->maneuver));
        //printHeap(openList);

        if (closeEnough(nodeOfInterest, path->endPoint)){ //check if the current node is the goal
            success = 1; //set the success flag
            break; //exit the core loop
        } 

        //create the list of kinematically feasible neighbors
        heap_t *kN = kinematicNeighbors(nodeOfInterest, path->vehicle);
        //printHeap(kN);
        //trim the neighbor list by fesibility (bounds and other geofences)
        heap_t *N = trimNeighbors(map, kN, path->vehicle);
        //printHeap(N);
        freeHeap(kN); //original neighbor list needs to be freed
        while(N->size > 0) { //need to check the list of feasible neighbors to make sure they haven't already been explored
            state_t *neighbor = extract(N); //pull off a neighbor
            if(neighbor == NULL) continue; //error-check. Should not be triggered.
            calcCosts(neighbor, path->endPoint, path->vehicle); //calculate the costs of this neighbor
            int inList = 0; //flag set if the node is found in either list
            for(int i = 0; i < closedCount; i++) { //search the entire closed list
                if(closeEnoughState(neighbor, closedList[i])) { //use utility function
                    free(neighbor); //free the neighbor here if it's found in the closed list
                    inList = 1; //set the flag
                    break; //exit the loop
                }
            }
            if(inList == 0) { //check if it was not found in the closed list
                //scan the open list as well
                for(int i = 0; i < openList->capacity; i++) {
                    state_t* state = openList->arr[i];
                    if(state != NULL) {
                        if(closeEnoughState(state, neighbor)) {
                            free(neighbor);
                            inList = 1;
                            break;
                        }
                    }
                }
            }
            if(inList == 1) continue; //if it was found in the closed list, continue to the next neighbor.
            insert(openList, neighbor); //if the neighbor was not found in the closed list, add it to the open list.
        }
        freeHeap(N); //free the heap of viable neighbors, which is empty at this point
        closedList[closedCount++] = nodeOfInterest; //increment the size of the closed list, and add the node that was just explored.
    }

    path->id = global_id_counter++; //assign and increment global counter

    if(success == 1) { //if the success flag was set
        //update the path struct accordingly.
        path->solved = 1; //path was found successfully. Set flag on path struct
        path->finalg = nodeOfInterest->g; //record final node path cost for stats
        path->solution = nodeOfInterest; //point the path object to the linked list of states that represents the solution.
        while(nodeOfInterest != NULL) { //backtrack through the found solution
            nodeOfInterest->isFinalSolution = 1; //set the flag on the states that are part of the solution so they are not freed by accident during cleanup
            nodeOfInterest = nodeOfInterest->previous;

        }
    } else {
        path->solved = -1; //path was not found successfully.
        path->solution = NULL; //no solution found.
    }

    cleanHeap(openList); //cleanup routine to free all the states left to be explored after solution was found
                         //in the case that the algorithm failed, this does nothing
    freeHeap(openList); //free the list itself
    cleanList(closedList, closedCount); //clean the closed list. Skips all the nodes that were part of the solution.
    free(closedList); //free the closed list itself.

    clock_t endTime = clock(); //stop the clock
    path->timeToSolve_ms =  1000*((double)(endTime - beginTime))/CLOCKS_PER_SEC; //record solution time

 }

void printPath(path_t *path) {
    //Simple printout routine for the path. Useful for debugging.
    printf("Path Information:\n");
    printf("    ID:%d   tau0:%d\n", path->id, path->ideal_tau_start);
    printf("    Start Point: ");
    printPoint(path->startPoint);
    printf("\n    End Point: ");
    printPoint(path->endPoint);
    printf("\n Solution Found: %s", path->solved == 1 ? "Yes" : "No");
    if(path->solved == 1) {
        printf("\n     State Sequence:\n");
        printStateSequence(path->solution); //calls helper function
    }
    printf("\n Solve Time (ms): %.2f\n", path->timeToSolve_ms);
}

void exportPath(path_t *path) {
    char buffer[50];
    sprintf(buffer, "path%d.csv",path->id); //uses unique file name based on assigned id
    //CSV file format to be convenient
    FILE* out = fopen(buffer, "w"); //open file for writing
    if(out == NULL) {
        fprintf(stderr, "Failed to open file for detailed path writing. \n");
        exit(EXIT_FAILURE);
    }
    fprintf(out, "x,y,z,t,v,psi,h,g,maneuver\n");
    // columns are x,y,z,t positions, velocity, heading, heuristic, path cost and maneuver type
    state_t *state = path->solution; // pull out the head of the solution linked list
    while(state != NULL) { //iteratively loop through each one
        fprintf(out, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n",state->position.x, state->position.y, state->position.z, state->time, state->velocity, state->heading, state->h, state->g, maneuver_to_string(state->maneuver));
        state = state->previous;
    }
    fclose(out); //close file
}

void exportPathShort(FILE* out, path_t *path) {
    //short version export of a path's information
    //this is used in batch/monte carlo runs to gather statistics
    fprintf(out, "%d, %d, %.2f, %.2f, %.2f\n", path->id, path->ideal_tau_start, path->initialh, path->finalg, path->timeToSolve_ms);
}

void freePath(path_t *path) {
    // Releases all the memory associated with a path
    // This can be safely done after writing the path into the tensor.
    freeStateSequence(path->solution); //call iterative free for the linked list of states which is the solution
    free(path); //free the remaining memory
}

void freeStateSequence(state_t *state) {
    if(state == NULL) return; //base case - null state
    freeStateSequence(state->previous); //recursive call on previous state
    free(state); //free after recursive call efficiently frees the entire linked list
}

void printPoint(point_t point) {
    printf("(x: %.2f, y: %.2f, z:%.2f)", point.x, point.y, point.z); //prints out the a point's coordinates.
}

void printStateSequence(state_t *state) {
    while(state != NULL) { // iterative print of states in a linked-list sequence
        printf("        State: ");
        printPoint(state->position);
        printf(" t: %.2f, v: %.2f, psi: %.2f, h: %.2f, g: %.2f, maneuver: %s\n", state->time, state->velocity, state->heading, state->h, state->g, maneuver_to_string(state->maneuver));
        state = state->previous;
    }
}

const char *maneuver_to_string(int maneuver) {
    switch (maneuver) {
        case CRUISE: return "CRUISE";
        case ACCELERATE: return "ACCELERATE";
        case DECELERATE: return "DECCELERATE";
        case LEFT: return "LEFT";
        case RIGHT: return "RIGHT";
        case CLIMB: return "CLIMB";
        case DESCEND: return "DESCEND";
        case YETTOMOVE: return "YETTOMOVE";
        default: return "UNKNOWN";
    }
}

heap_t* createHeap(int capacity) {
    heap_t *h = (heap_t*)calloc(1,sizeof(heap_t)); //allocate memory for heap object

    h->size = 0; //initial size is zero
    h->capacity = capacity; // set capacity

    h->arr = (state_t**)calloc(capacity,sizeof(state_t*)); //allocate memory for list of state pointers
    return h; //return heap object
}

void insert(heap_t *h, state_t* value) {
    //standard minheap implmentation
    //insert new state and re-sort
    if(h->size >= h->capacity) {
        fprintf(stderr, "Heap Capacity Exceeded\n");
        exit(EXIT_FAILURE);
    }

    h->arr[h->size] = value; //insert new value at end

    int i = h->size;
    h->size++; //increment size

    //bubble up
    while (i != 0 && f_value(h->arr[i]) < f_value(h->arr[(i - 1) / 2])) {
        swap(&h->arr[i], &h->arr[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

state_t* extract(heap_t *h) {
    //standard minheap implementation
    state_t *min = h->arr[0]; //lowest cost node is at the top
    h->arr[0] = h->arr[h->size - 1];
    h->size--; //decrement size

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

void printState(state_t *state) {
    printf("    State: "); //utility print state out
    printPoint(state->position);
    printf(" t: %.2f, v: %.2f, psi: %.2f, h: %.2f, g: %.2f, maneuver: %s\n", state->time, state->velocity, state->heading, state->h, state->g, maneuver_to_string(state->maneuver));
}

void printHeap(heap_t *h) {
    //prints out contents of a heap without disturbing it for debugging
    if (!h || h->size == 0) {
        printf("Heap is empty.\n");
        return;
    }

    printf("Min-Heap Contents:\n");
    for (int i = 0; i < h->size; i++) {
        printf("Index %d: ", i);
        if (h->arr[i]) {
            printState(h->arr[i]);
        } else {
            printf("NULL");
        }
        printf("\n");
    }

}

void swap(state_t **a, state_t **b) {
    state_t *temp = *a;
    *a = *b;
    *b = temp;
}

void resetGlobalIdCounter() {
    global_id_counter = 0;
}

float f_value(state_t* state) {
    return state->g + state->h;
}

void freeHeap(heap_t *h) {
    free(h->arr); //free heap array
    free(h);
}

void cleanHeap(heap_t *h) {
    while(h->size > 0) { //iteratively pop states of the heap and free them
        state_t *state = extract(h);
        free(state);
    }
}

void cleanList(state_t **list, int size) {
    for(int i = 0; i < size; i++) { //iteratively loops through the list and frees everything that's not part of the final solution
        state_t *state = list[i];
        if(!state->isFinalSolution) free(state); //check state flag and free if not part of final solution
    }
}