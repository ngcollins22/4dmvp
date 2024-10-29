/*

    4DMVP Project for Collins Aerospace
    Author: Nathan Collins
    Date Created: 8/18/24

*/

#include "pathfind.h"

/*
    TODO: Create a map, populate with obstacles, write out to a file
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

int main() {
    map_t* map = emptyMap(8, 4, 2, 1);
    FILE* out = fopen("test.txt", "w");


    point_t p1;
    point_t p2;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;

    p2.x = 5;
    p2.y = 2;
    p2.z = 2;

    path_t path = pathFind(map, p1, p2);
    printPath(path);
    /*
    setValueAt(map->occupancytensor,5,2);
    setValueAt(map->occupancytensor,6,1);
    for(int i = 0; i < 8*4*2; i++) {
        setValueAt(map->occupancytensor, i, i%4);
    }

    printf("%d", getIndex(map->occupancytensor, 5, 0, 0, 0));
    */
    exportMap(map, out);
    freePath(path);
    freeMap(map);
    fclose(out);
}