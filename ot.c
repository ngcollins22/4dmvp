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

    map->timeStep_s = 1; //arbitrary
    map->unitWidth_m = 15; //arbitrary

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

/*
    Create a new tensor with given dimensions
*/
ot_t *createTensor(int w, int l, int h, int tf) {

    ot_t *ot = (ot_t*) malloc(sizeof(ot_t));
    ot->W = w;
    ot->H = h;
    ot->L = l;
    ot->tf = tf;

    char* tensor = (char*) calloc(((2*w*l*h*(tf+1))/8), sizeof(char)); //each one needs 2 bits, this MAY overflow by up to 6 bits

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
    int volume = (ot->W)*(ot->L)*(ot->H);
    int volumeIndex = volume * t; //index at which this timestep's map starts
    int area = (ot->W)*(ot->L);
    int areaIndex = area*z; //index at which this x-y map starts
    int lineIndex = (ot->W)*y; //index at which this x sequence is at
    return volumeIndex + areaIndex + lineIndex + x;
}

int getValueAt(ot_t *ot, int index) {
    uint8_t arrayIndex = index/4;
    uint8_t r = index - arrayIndex*4;
    uint8_t mask = 192 >> (2*r); //mask to pull out appropriate uint8
    //printf("mask: %b\n", mask);
    //printf("whatever this is: %b\n", (ot->occupancymap[arrayIndex]) & mask);
    uint8_t val = ((ot->occupancymap[arrayIndex]) & mask) >> (6 - 2*r); //god knows
    //printf("%d", val);
    return val;
}

void setValueAt(ot_t *ot, int index, int value) {
    uint8_t arrayIndex = index/4;
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
    fprintf(out,"(%d,%d,%d,%d)\n",map->W, map->L, map->H,map->occupancytensor->tf); //header
    exportTensor(map->occupancytensor, out);
}

void exportTensor(ot_t *ot, FILE* out) {
    for(int t = 0; t < ot->tf; t++) { //for each temporal index
        fprintf(out, "[\n");
        for(int z = 0; z < ot->H; z++) { //for each z value
            fprintf(out, "[\n");
            for(int y = 0; y < ot->L; y++) { //for each y value
                fprintf(out, "[");
                for(int x = 0; x < ot->W; x++) { //for each x value
                    int index = getIndex(ot,x,y,z,t); //pull index
                    int val = getValueAt(ot, index); //pull actual 2bit value
                    fprintf(out, "%d%s", val, (x == (ot->W -1)) ? "" : ",");
                }
                fprintf(out, "]%s\n", (y == (ot->L -1)) ? "" : ",");
            }
             fprintf(out, "]%s\n", (z == (ot->H -1)) ? "" : ",");
        }
        fprintf(out, "]%s\n",  (t == (ot->tf - 1)) ? "" : ",");
    }
}

void extendTimeHorizon(ot_t *ot, int n) {
    //extend time horizon by n timesteps    
    //how many bytes are currently allocated?
    //how many more bytes do I need
    //realloc the tensor -> will continous
    //memset everything to zeros
}

void leftTruncate(ot_t *ot);