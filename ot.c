#include "ot.h"

/*
    Source file containing functions and utilities for maps and occupancy tensors
*/

/*
    Create a new blank map with given spatial dimensions
        Assumed reference time to be zero utc, same with startTime

*/
map_t *emptyMap(int w, int l, int h, int tf) {
    map_t *map = (map_t*) calloc(1, sizeof(map_t));
    map->W = w;
    map->L = l;
    map->H = h;

    time_t current_time;
    struct tm *gmt_time;
    
    // Get current time
    time(&current_time);
    
    // Convert current time to UTC (GMT)
    gmt_time = gmtime(&current_time);
    
    // Set hours, minutes, seconds to 0 to get midnight UTC
    gmt_time->tm_hour = 0;
    gmt_time->tm_min = 0;
    gmt_time->tm_sec = 0;
    
    // Convert back to time_t
    time_t midnight_utc = mktime(gmt_time);

    map->referenceTime = midnight_utc;
    map->startTime = midnight_utc;

    map->tau = 1; //arbitrary
    map->chi = 15; //arbitrary

    ot_t* ot = createTensor(w,l,h,tf);
    map->occupancytensor = ot;

    return map;
}

/*
    Pull the value of a map at spatial and temporal coordinates
        Returns -1 if the index does not exist
*/
int pull(map_t *map, int x, int y, int z, int t) {
    int n = getIndex(map->occupancytensor, x, y, z, t);
    return getValueAt(map->occupancytensor, n);
}

void push(map_t *map, int x, int y, int z, int t, int val) {
    int n = getIndex(map->occupancytensor, x, y, z, t);
    //int oldVal = pull(map, x, y, z, t);
    //printf("Pushing a %d into a %d at (%d,%d,%d,%d)\n", val, oldVal, x, y, z, t);
    //anything may replace a zero
    //a 1 may replace a 2

    int oldVal = pull(map, x, y, z, t);    
    if(oldVal == 0 && val != 0) { //something replacing a zero
        setValueAt(map->occupancytensor, n, val);
    } else if(oldVal == 2 && val == 1) { // a 1 is replacing a two
        setValueAt(map->occupancytensor, n, val);
    } // a 1 is immutable
}

/*
    Create a new tensor with given dimensions
*/
ot_t *createTensor(int w, int l, int h, int tf) {
    ot_t *ot = (ot_t*) malloc(sizeof(ot_t));
    if (!ot) {
        fprintf(stderr, "Failed to allocate memory for tensor\n");
        exit(EXIT_FAILURE);
    }
    ot->W = w;
    ot->H = h;
    ot->L = l;
    ot->tf = tf;
    uint8_t* tensor = (uint8_t*) calloc((((ot->W * ot->L * ot->H * (ot->tf + 1)) + 3) / 4), sizeof(char)); //each one needs 2 bits, this MAY overflow by up to 6 bits
    if (!tensor) {
        fprintf(stderr, "Failed to allocate memory for occupancy map\n");
        free(ot);
        exit(EXIT_FAILURE);
    }
    //printf("Creating tensor with size: %d", (((ot->W * ot->L * ot->H * (ot->tf + 1)) + 3) / 4));
    ot->occupancymap = tensor;
    return ot;
}

void freeMap(map_t *map) {
    freeTensor(map->occupancytensor);
    free(map);
}

void freeTensor(ot_t *ot) {
    free(ot->occupancymap);
    free(ot);
}

int getIndex(ot_t *ot, int x, int y, int z, int t) { //does this work??
    /**/
    int volume = (ot->W)*(ot->L)*(ot->H);
    int volumeIndex = volume * t; //index at which this timestep's map starts
    int area = (ot->W)*(ot->L);
    int areaIndex = area*z; //index at which this x-y map starts
    int lineIndex = (ot->W)*y; //index at which this x sequence is at
    //printf("(%d,%d,%d,%d) -> %d\n", x, y, z, t, volumeIndex + areaIndex + lineIndex + x);
    if (x < 0 || x >= ot->W || y < 0 || y >= ot->L || z < 0 || z >= ot->H || t < 0 || t >= ot->tf) {
        return -1; // Sentinel value indicating out-of-bounds
    }
    return volumeIndex + areaIndex + lineIndex + x;
}

int getValueAt(ot_t *ot, int index) {
    int arrayIndex = index/4;
    int r = index - arrayIndex*4;
    uint8_t mask = 192 >> (2*r); //mask to pull out appropriate uint8
    //printf("whatever this is: %b\n", (ot->occupancymap[arrayIndex]) & mask);
    uint8_t val = ((ot->occupancymap[arrayIndex]) & mask) >> (6 - 2*r); //god knows
    //printf("%d", val);
    //if(val == 2) printf("isolated value: %b w/ r = %d at index %d\n", val, r, index);
    return val;
}

void setValueAt(ot_t *ot, int index, uint8_t value) {
    //printf("Setting %d to %d\n", index, value);
    int arrayIndex = index/4;
    //printf("arrayIndex:%d\n", arrayIndex);
    int r = index - arrayIndex*4;
    //printf("r:%d\n", r);
    uint8_t mask1 = 255 ^ (192 >> (2*r));
    //printf("mask1 flip:%b\n", (uint8_t)~mask1);
    uint8_t mask2 = ((uint8_t)~mask1) & (value << (6 - 2*r));
    //printf("isolated value:%b\n", value << (6 - 2*r));
    //printf("mask2:%b\n", mask2);
    uint8_t quadValue = ot->occupancymap[arrayIndex];
    //printf("%b\n", quadValue);
    uint8_t newQuadValue = (quadValue & mask1) | mask2;
    ot->occupancymap[arrayIndex] = newQuadValue;
    //printf("%b\n", newQuadValue);
}

void exportMap(map_t *map, FILE *out) {
    printf("Writing to file... ");
    fflush(stdout);
    fprintf(out,"(%d,%d,%d,%d)\n",map->W, map->L, map->H,map->occupancytensor->tf); //header
    //exportTensor(map->occupancytensor, out);
    ot_t *ot = map->occupancytensor;
    //printf(ot);
    for(int t = 0; t < ot->tf; t++) { //for each temporal index
        fprintf(out, "[\n");
        for(int z = 0; z < ot->H; z++) { //for each z value
            fprintf(out, "[\n");
            for(int y = 0; y < ot->L; y++) { //for each y value
                fprintf(out, "[");
                for(int x = 0; x < ot->W; x++) { //for each x value
                    //int index = getIndex(ot, x, y, z, t); //pull index
                    //int val = getValueAt(ot, index); //pull actual 2bit value
                    //if(val == 1) printf("penis");
                    int rawVal = pull(map, x, y, z, t);
                    //if(rawVal == 2) printf("2 encountered in export Map\n");
                    //printf("Raw value at (%d, %d, %d, %d): %d\n", x, y, z, t, rawVal);
                    fprintf(out, "%d%s", rawVal, (x == (ot->W - 1)) ? "" : ",");
                }
                fprintf(out, "]%s\n", (y == (ot->L -1)) ? "" : ",");
            }
             fprintf(out, "]%s\n", (z == (ot->H -1)) ? "" : ",");
        }
        fprintf(out, "]%s\n",  (t == (ot->tf - 1)) ? "" : ",");
    }
    printf(" Done\n");
}

void extendTimeHorizon(ot_t *ot, int n) {
    printf("Extending Tensor Time Horizon: \n");
    //extend time horizon by n timesteps
    long oldSize = (((ot->W * ot->L * ot->H * (ot->tf + 1)) + 3) / 4);
    printf("    Previous Size (bytes): %ld\n", oldSize);
    long newSize = (((ot->W * ot->L * ot->H * (ot->tf + n + 1)) + 3) / 4);
    printf("    New Size (bytes): %ld\n", newSize);

    //how many bytes are currently allocated?
    uint8_t* newMap = realloc(ot->occupancymap, newSize);
    if (!newMap) {
        fprintf(stderr, "Failed to extend tensor memory :( \n Get a better computer\n");
        exit(EXIT_FAILURE); // Or handle gracefully
    }
    ot->occupancymap = newMap;
    //memset everything to zeros
    //printf("Final Time = %d\n", ot->tf);

    memset(ot->occupancymap + oldSize, 0, newSize - oldSize);
    ot->tf = ot->tf + n;
}