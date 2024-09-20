/*

    4DMVP Project for Collins Aerospace
    Author: Nathan Collins
    Date Created: 8/18/24

*/

#include "flights.h"

/*
    TODO: Create a map, populate with obstacles, write out to a file
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

map_t* map;

int main() {

    //for(int i = 0; i < 100; i++) {
        map = create_blank_map(10, 10);
        //populate the map with obstacles
        populate_with_obstacles(map, 0.3);

        flightplan_t *flightplan1 = (flightplan_t*) malloc(sizeof(flightplan_t));

        chooseRandom(flightplan1, map, 1); 

        printplan(flightplan1, map); 

        node_t *startNode = pullIndex(map, flightplan1->start);
        startNode->status = OCCUPIED;
        node_t *endNode = pullIndex(map, flightplan1->end);
        displayNode(startNode);
        displayNode(endNode);

        calc_heuristic(map, pullIndex(map, flightplan1->start),  pullIndex(map, flightplan1->end));

        printf("start to finish heuristic (squared): %f ", pullIndex(map, flightplan1->start)->hcost);

        displayMap(map);

        double compTime;
        int success = pathfind(flightplan1, map, &compTime);
        printf(success ? "Path found" : "Path not found");
        free(flightplan1);

        displayMap(map);

        flightplan_t *flightplan2 = (flightplan_t*) malloc(sizeof(flightplan_t));

        chooseRandom(flightplan2, map, 2); 

        printplan(flightplan2, map); 

        startNode = pullIndex(map, flightplan2->start);
        startNode->status = OCCUPIED;
        endNode = pullIndex(map, flightplan2->end);
        displayNode(startNode);
        displayNode(endNode);

        calc_heuristic(map, pullIndex(map, flightplan2->start),  pullIndex(map, flightplan2->end));

        printf("start to finish heuristic (squared): %f ", pullIndex(map, flightplan2->start)->hcost);

        displayMap(map);

        pathfind(flightplan2, map, &compTime);
        free(flightplan2);

        displayMap(map);

        //known problems
            //undefined behavior if a path can't be found
            //displaying use some work
            //



        displayMap(map);


        displayMap(map); 
        
        freeGrid(map); 
        free(map); 
        //printf("Iteration %d/100 Complete \n", i+1);
    //}
    /*
    printplan(flightplan, map); 

    node_t *startNode = pullIndex(map, flightplan->start);
    startNode->status = OCCUPIED;
    node_t *endNode = pullIndex(map, flightplan->end);
    endNode->status = OCCUPIED;
    displayNode(startNode);
    displayNode(endNode);
    calc_heuristic(map, pullIndex(map, flightplan->start),  pullIndex(map, flightplan->end));

    printf("start to finish heuristic (squared): %f ", pullIndex(map, flightplan->start)->hcost);

    displayMap(map); 
    */

    //freeGrid(map); 
    //free(map); 
    }