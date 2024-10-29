#include "pathfind.h"

const float WT = 0.5;
const float WG = 1 - WT;
const float GEOFENCE_RADIUS = 3;


node_t** getNeighbors(map_t *map, node_t* node, point_t goal, int* numberOfNeighbors) {
    //26 neighbors
    //neighbors are
    //all 3D adjacent cubes, plus the "hover" option
    int i = 0;
    int numNeighbors = 0;
    node_t** neighborList = NULL;
    for(int dz = -1; dz <= 1; dz++) {
        for(int dy = -1; dy <= 1; dy++) {
            for(int dx = -1; dx <= 1; dx++) {
                int moveType = abs(dx) + abs(dy) + abs(dz);
                point_t newPoint;
                newPoint.x = node->point.x + dx;
                newPoint.y = node->point.y + dy;
                newPoint.z = node->point.z + dz;
                if(newPoint.x < 0 || newPoint.y < 0 ||newPoint.z < 0 || newPoint.x >= map->W || newPoint.y >= map->W || newPoint.z >= map->W) continue;
                if(moveType == 0) { //hover move!
                    addToList(&neighborList, &numNeighbors, makeNode(node, newPoint, WG*0 + 2*WT*1, 1, goal));
                } else if (moveType == 1) { //orthogonal traverse
                    addToList(&neighborList, &numNeighbors, makeNode(node, newPoint, WG*1 + WT*1, 1, goal));
                } else if (moveType == 2) { //2D diagnal traverse
                    addToList(&neighborList, &numNeighbors, makeNode(node, newPoint, WG*1.4 + WT*1.4, 1.4, goal));
                } else if (moveType == 3) { //3D diagnal traverse
                    addToList(&neighborList, &numNeighbors, makeNode(node, newPoint, WG*1.7 + WT*1.7, 1.7, goal));
                }
                i++;
            }
        }
    }
    *numberOfNeighbors = numNeighbors;
    return neighborList;
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
        int nl = getIndex(ot, wp.location.x, wp.location.y, wp.location.z, (int) floor(wp.time));
        int nu = getIndex(ot, wp.location.x, wp.location.y, wp.location.z, (int) ceil(wp.time));
        setValueAt(ot, nl, 1);
        setValueAt(ot, nu, 1);
        //printf("Setting a value \n");
    }
}

void addWaypointToTensor(ot_t *ot, wp_t wp) {

}

float dist(point_t p1, point_t p2) {

    int dx = p1.x - p2.x;
    int dy = p1.y - p2.y;
    int dz = p1.z - p2.z;

    return sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
}

extern node_t* makeNode(node_t *prev, point_t point, float dg, float dt, point_t goal) {
    node_t* node = (node_t*) calloc(1, sizeof(node_t));
    node->point = point;
    node->gcost = prev->gcost + dg;
    node->t = prev->t + dt;
    node->hcost = dist(point, goal);
    node->prev = prev;
}

extern node_t* makeStarterNode(point_t point, point_t goal) {
    node_t* node = (node_t*) calloc(1, sizeof(node_t));
    node->point = point;
    node->gcost = 0;
    node->t = 0;
    node->hcost = dist(point, goal);
    node->prev = NULL;
    return node;
}

point_t point(int x, int y, int z) {
    point_t p;
    p.x = x;
    p.y = y;
    p.z = x;
    return p;
}

path_t pathFind(map_t *map, point_t start, point_t end) {
    //start the timer
    clock_t beginTime = clock();
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
        //printf("A* iteration start: ");
        int lowestF = __INT_MAX__;
        int index = -1;
        for(int i = 0; i < openCount; i++) {
            node_t *currentNode = openList[i];
            if(fcost(currentNode) < lowestF) {
                lowestF = fcost(currentNode);
                index = i;
                nodeOfInterest = currentNode;
            }
        }
            //check if that node is the goal
                //if yes, win
        if(match(nodeOfInterest->point, end)) {
            success = 1;
            break;
        }
            
        int numNeighbors = 0;
        node_t **neighborList = getNeighbors(map, nodeOfInterest, end, &numNeighbors);
        
        for(int i = 0; i < numNeighbors; i++) {
            node_t *neighbor = neighborList[i];
            int alreadySeen = 0;
            for(int j = 0; j < closedCount; j++) {
                if(match(neighbor->point, closedList[j]->point) && closedList[j]->t == neighbor->t) {
                    alreadySeen = 1;
                    break;
                }
            }
            if(alreadySeen == 1) {
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
            //get all neighbors of this node, for each
                //check if on closed list -> yes, skip
                //if I exceed the current map's final time -> everything is game
                //if not
                    //integer time
                        //check value
                            // 0->add to open list
                            // 1->do not add to open list
                            // 2->perform space check to determine if we add or not
                    //if not integer time
                        //check floor and ceiling
                            //both zeros -> add to open list
                            //either is a 1 -> do not add to open list
                            //one is 2 -> perform space check, yes -> add to list
                            //both are 2 -> perform space check, both yes -> add to list
        
        removeFromList(&openList, &openCount, index);
        addToList(&closedList, &closedCount, nodeOfInterest);
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

wp_t nodeToWaypoint(node_t *node) {
    wp_t wp;
    wp.location.x = node->point.x;
    wp.location.y = node->point.y;
    wp.location.z = node->point.z;
    wp.time = node->t;
    return wp;
}

int spaceCheck(map_t *map, node_t *node) {
    //check if I have space to place a geofence here

    //find where I would actually be at this point in time
    int tLower = floor(node->t);
    int tHigher = ceil(node->t);
    if(tLower - tHigher < 0.05) { //avoid some floating point errors
        // I'm on "integer time"
        int val = pull(map, node->point.x, node->point.y, node->point.z, tLower);
        if(val == 0) {
            //free node! 
            return 1;
        } else if (val == 1) {
            return 0;
        } else if (val == 2) {
            //I need to do a detailed check
            return 1; //REPLACE
        }
    } else {
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
    //if I have all zeros, I'm good

    //if I have a 1, I'm not good


    //if I have a 2
        //collect every place where I would place a 1 if I were to be here
        //if any of them are already 1s, im cooked
    
}

int sphereCast(ot_t *ot, point_t p, float t) {
    //Find all the point with a cartesian distance less than my tolerance away
}

float fcost(node_t *node) {
    return node->hcost + node->gcost;
}

int match(point_t p1, point_t p2) {
    return (p1.x == p2.x) && (p1.y == p2.y) && (p1.z == p2.z);
}

void removeFromList(node_t ***list, int *size, int index) {
    //shift every index forward back by 1
    for(int i = index; i < (*size)-1; i++) {
        (*list)[i] = (*list)[i+1]; //point to the next one instead
    }
    //decrement the size
    *size = *size -1;
    //realloc the list
    *list = (node_t**) realloc(*list, sizeof(node_t*) * (*size));
}

void addToList(node_t ***list, int *size, node_t *node) {
    //increment the size
    *size = *size + 1;
    *list = (node_t**) realloc(*list, sizeof(node_t*) * (*size));
    //add the node as the last index
    (*list)[(*size)-1] = node;
}

int contains(node_t **list, int *size, node_t *node) {
    int found = 0;
    if(*size == 0) return 0;
    for(int i = 0; i < *size;i++) {
        if(list[i] == node) {
            found = 1;
            break;
        }
    }
    return found;
}

void printPath(path_t path) {
    printf("Path: (%d,%d,%d) -> (%d,%d,%d) (%d steps) in %f epochs, solved in %dms: \n", 
        path.waypoints[0].location.x,
        path.waypoints[0].location.y,
        path.waypoints[0].location.z,
        path.waypoints[path.pathLength-1].location.x,
        path.waypoints[path.pathLength-1].location.y,
        path.waypoints[path.pathLength-1].location.z,
        path.pathLength,
        path.tf,
        path.timeToSolvems);
    for(int i = 0; i < path.pathLength; i++) {
        printf("(%d,%d,%d) @ t = %f\n", path.waypoints[i].location.x,
        path.waypoints[i].location.y,
        path.waypoints[i].location.z, path.waypoints[i].time);
    }
}

void freeList(node_t **list, int size) {
    for(int i = 0; i < size; i++) {
        node_t *l = list[i];
        free(l);
    }
}

void freePath(path_t path) {
    free(path.waypoints);
}

