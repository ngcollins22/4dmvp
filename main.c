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

    path_t *path = (path_t*)calloc(1, sizeof(path_t));

    vehicle_t vehicle = {
        .hdot = 1,
        .psidot = 0.1,
        .R = 2,
        .vdot = 0.05,
        .vpref = 2,
        .vstall = 1
    };

    path->vehicle = vehicle;
    path->ideal_tau_start = 0;
    point_t start = {.x = 0, .y = 0, .z = 0};
    point_t end = {.x = 16, .y = 4, .z = 9};
    path->startPoint = start;
    path->endPoint = end;
    pathFind(map, path);
    printPath(path);
    writePath(map, path);
    freePath(path);
    //point_t p1;
    //freePath(path2);
    exportMap(map, out);
    freeMap(map);
    fclose(out);
}