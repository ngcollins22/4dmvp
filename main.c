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

void dumpTensorAsBitsToFile(ot_t *ot, const char *filename) {
    FILE *out = fopen(filename, "w");  // Open in text mode for easy readability
    if (!out) {
        fprintf(stderr, "Error: Could not open file %s for writing\n", filename);
        return;
    }

    size_t size = ((ot->W * ot->L * ot->H * (ot->tf + 1) + 3) / 4);  // Total bytes in occupancymap
    for (size_t i = 0; i < size; i++) {
        // Retrieve each 2-bit value within the current byte
        for (int shift = 6; shift >= 0; shift -= 2) {
            uint8_t value = (ot->occupancymap[i] >> shift) & 0x03;  // Isolate 2 bits
            switch (value) {
                case 0: fprintf(out, "00"); break;
                case 1: fprintf(out, "01"); break;
                case 2: fprintf(out, "10"); break;
                case 3: fprintf(out, "11"); break;
            }
        }
        fprintf(out, "\n");  // Newline after each byte for readability
    }

    fclose(out);
    printf("Tensor dumped to %s as 1 and 0 values.\n", filename);
}


int main() {
    map_t* map = emptyMap(20, 20, 20, 1);
    FILE* out = fopen("test.txt", "w");

    //point_t p1;
    //point_t p2;
    //p1.x = 0;
    //p1.y = 0;
    //p1.z = 0;
    //p2.x = 9;
    //p2.y = 9;
    //p2.z = 9;
    
    for(int i = 0; i < 3; i++) {
        srand((unsigned int)clock());

        point_t p1;
        point_t p2;
        p1.x = rand() % map->L;
        p1.y = rand() % map->W;
        p1.z = rand() % map->H;

        p2.x = rand() % map->L;
        p2.y = rand() % map->W;
        p2.z = rand() % map->H;

        path_t path = pathFind(map, p1, p2);
        printPath(path);
        addPathToTensor(map->occupancytensor, &path);
        freePath(path);
    }
    
    //path_t path = pathFind(map, p1, p2);
    //printPath(path);
    //addPathToTensor(map->occupancytensor, &path);
    //freePath(path);
    exportMap(map, out);
    //freePath(path2);
    freeMap(map);
    fclose(out);
}