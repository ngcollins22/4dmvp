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

const int pace = 3;
int debugging = 0;
FILE* statFile;

void monteCarloIteration(int charLength, int charHeight, int maxIter) {
    map_t *map = emptyMap(charLength, charLength, charHeight, 1);
    FILE* tensorOut = fopen("tensor.csv", "w");
    FILE* shortPathInfo = fopen("paths.csv","w");
    fprintf(shortPathInfo, "ID, IdealT0, H0, Gf, TTS (ms)\n");
    printf("Monte Carlo Iteration Start: (%dX%dX%d) \n", charLength, charLength, charHeight);
    vehicle_t vehicle = {
        .hdot = 1,
        .psidot = 0.1,
        .R = 2,
        .vdot = 0.05,
        .vpref = 2,
        .vstall = 1
    };
    int tau_randomizer = 0;
    for(int i = 0; i < maxIter; i++) {
        printf("Iteration #%d\n", i+1);
        srand(clock());
        path_t *path = (path_t*)calloc(1,sizeof(path_t));
        path->vehicle = vehicle;
        path->ideal_tau_start = tau_randomizer;
        //tau_randomizer += rand() % pace;
        int z = 5;
        point_t start = {.x = rand() % charLength, .y = rand() % charLength, .z = z};
        point_t end = {.x = rand() % charLength, .y = rand() % charLength, .z = z};
        path->startPoint = start;
        path->endPoint = end;
        pathFind(map, path);
        if(path->solved == -1) {
            printf("Path #%d failed to solve. Monte Carlo exiting.\n", path->id);
            exportPathShort(shortPathInfo, path);
            freePath(path);
            break;
        }
        exportPathShort(shortPathInfo, path);
        exportPathShort(statFile, path);
        exportPath(path);
        //if(debugging == 1) printPath(path);
        writePath(map, path);
        freePath(path);
        if(i == maxIter-1) {
            printf("Maximum Iteration count reached. Monte Carlo exiting.\n");
        }
    }
    resetGlobalIdCounter();
    //exportMap(map, tensorOut);
    fclose(shortPathInfo);
    fclose(tensorOut);
    freeMap(map);
}

int main(int argc, char** argv) {

    for(int i = 0; i < argc; i++) {
        if(strcmp(argv[i],"-g") != 0) {
            debugging = 1;
        }
    }

    statFile = fopen("stats.csv","w");
    for(int i = 0; i < 1000; i ++) {
        monteCarloIteration(100, 20, 100);
    }
    fclose(statFile);
    // map_t* map = emptyMap(100, 100, 100, 1);
    // FILE* out = fopen("test.txt", "w");

    // path_t *path = (path_t*)calloc(1, sizeof(path_t));
    // path_t *path2 = (path_t*)calloc(1, sizeof(path_t));


    // vehicle_t vehicle = {
    //     .hdot = 1,
    //     .psidot = 0.1,
    //     .R = 2,
    //     .vdot = 0.05,
    //     .vpref = 2,
    //     .vstall = 1
    // };

    // path->vehicle = vehicle;
    // path->ideal_tau_start = 0;
    // point_t start = {.x = 10, .y = 20, .z = 12};
    // point_t end = {.x = 40, .y = 40, .z = 19};
    // path->startPoint = start;
    // path->endPoint = end;
    // pathFind(map, path);
    // printPath(path);
    // writePath(map, path);
    // path2->vehicle = vehicle;
    // path2->ideal_tau_start = 4;
    // point_t start2 = {.x = 20, .y = 10, .z = 12};
    // point_t end2 = {.x = 40, .y = 40, .z = 12};
    // path2->startPoint = start2;
    // path2->endPoint = end2;
    // pathFind(map, path2);
    // printPath(path2);
    // writePath(map, path2);
    // freePath(path);
    // freePath(path2);
    // exportMap(map, out);
    // freeMap(map);
    // fclose(out);
}