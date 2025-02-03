#include "ot.h"

/*
    Source file containing functions and utilities for maps and occupancy tensors
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

    map->tau = 1; //seconds
    map->chi = 15; //meters

    ot_t* ot = createTensor(w,l,h,tf); //utility function to create the tensor struct
    map->occupancytensor = ot;

    return map;
}

int pull(map_t *map, int x, int y, int z, int t) {
    //actual forward-facing function called from other files
    int n = getIndex(map->occupancytensor, x, y, z, t); //find the two-bit index associated with the coordinate given
    return getValueAt(map->occupancytensor, n); //get the value at that two-bit index
}

void push(map_t *map, int x, int y, int z, int t, int val) {
    //this is the actual forward-facing function called from other files
    int n = getIndex(map->occupancytensor, x, y, z, t); //find the two-bit index associated with the coordinate given

    int oldVal = pull(map, x, y, z, t); //find the value currently at the index
    /*
        Replacement rules:
        A 1 may replace a 2
        A 1 may replace a 0
        A 2 may replace a 0

        Priority: 1 > 2 > 0
    */
    if(oldVal == 0 && val != 0) { //The old value was a zero
        setValueAt(map->occupancytensor, n, val); //call the actual set value function
    } else if(oldVal == 2 && val == 1) { // a 1 is replacing a two
        setValueAt(map->occupancytensor, n, val);
    } // a 1 is immutable
}

ot_t *createTensor(int w, int l, int h, int tf) {
    ot_t *ot = (ot_t*) calloc(1,sizeof(ot_t)); //allocate memory for tensor struct
    if (!ot) { //memory failure check
        fprintf(stderr, "Failed to allocate memory for tensor\n");
        exit(EXIT_FAILURE);
    }
    ot->W = w; //set metadata
    ot->H = h;
    ot->L = l;
    ot->tf = tf;
    uint8_t* tensor = (uint8_t*) calloc((((ot->W * ot->L * ot->H * (ot->tf + 1)) + 3) / 4), sizeof(char)); 
    /*
        Formula for size (in bytes) is four times the total number of indices, which is the product of L,W,H,tf
        Adding three and then integer dividing by four ensures we allocate enough memory to fit everything.
        It may oversize by up to 6 bits.
    */
    if (!tensor) { //memory failure check
        fprintf(stderr, "Failed to allocate memory for occupancy map\n");
        free(ot);
        exit(EXIT_FAILURE);
    }
    //printf("Creating tensor with size: %d", (((ot->W * ot->L * ot->H * (ot->tf + 1)) + 3) / 4));
    ot->occupancymap = tensor; //point the occupancy tensor object to the array
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

int getIndex(ot_t *ot, int x, int y, int z, int t) { 
    //This returns the two-bit index!
    int volume = (ot->W)*(ot->L)*(ot->H); //total 3D volume of the tensor's space
    int volumeIndex = volume * t; //index at which this timestep's map starts
    int area = (ot->W)*(ot->L); //area of an x-y slice
    int areaIndex = area*z; //index at which this x-y map starts
    int lineIndex = (ot->W)*y; //index at which this x sequence is at
    //printf("(%d,%d,%d,%d) -> %d\n", x, y, z, t, volumeIndex + areaIndex + lineIndex + x);
    if (x < 0 || x >= ot->W || y < 0 || y >= ot->L || z < 0 || z >= ot->H || t < 0 || t >= ot->tf) {
        return -1; // Sentinel value indicating out-of-bounds
    }
    return volumeIndex + areaIndex + lineIndex + x;
}

int getValueAt(ot_t *ot, int index) {
    //Note: the index is the index of the 2-bit value
    int arrayIndex = index/4; //find the index of the 8-bit integer containing the two bits we want to find
    int r = index - arrayIndex*4; //remainder - position within 8-bit integer 
    uint8_t mask = 192 >> (2*r); //Mask to isolate 2-bit value of interest in 8-bit int
    uint8_t val = ((ot->occupancymap[arrayIndex]) & mask) >> (6 - 2*r); //use the mask, and then bitshift to pull the 2-bit value down to the least significant bits.
    if(index == -1) return -1; //sentinel value check
    return val; //return the found value
}

void setValueAt(ot_t *ot, int index, uint8_t value) {
    //Note: the index is the 2-bit index
    //Value is assumed to be 2-bit integer
    int arrayIndex = index/4;  //find the index of the 8-bit integer containing the two bits we want to find
    int r = index - arrayIndex*4; //remainder - position within 8-bit integer.
    uint8_t mask1 = 255 ^ (192 >> (2*r)); //mask containing 1s everywhere except where the new 2-bit value belongs.
    uint8_t mask2 = ((uint8_t)~mask1) & (value << (6 - 2*r)); //inverts mask 1 and moves in the new assigned value
    uint8_t quadValue = ot->occupancymap[arrayIndex]; //old 8-bit integer at the array index
    uint8_t newQuadValue = (quadValue & mask1) | mask2; //uses the masks to write in the new value without affecting the old ones
    ot->occupancymap[arrayIndex] = newQuadValue; //returns the new 8-bit value to the array
}

void exportMap(map_t *map, FILE *out) {
    // Corresponding matlab code reads this data in.
    printf("Writing to file... "); //This routine can take a significant amount of time, printout to show user it's running
    fflush(stdout); //flush required
    fprintf(out,"(%d,%d,%d,%d)\n",map->W, map->L, map->H,map->occupancytensor->tf); //Header written to file to show dimensions of tensor
    ot_t *ot = map->occupancytensor;
    for(int t = 0; t < ot->tf; t++) { //for each temporal index
        fprintf(out, "[\n");
        for(int z = 0; z < ot->H; z++) { //for each z value
            fprintf(out, "[\n");
            for(int y = 0; y < ot->L; y++) { //for each y value
                fprintf(out, "[");
                for(int x = 0; x < ot->W; x++) { //for each x value
                    int rawVal = pull(map, x, y, z, t); //pull the value at these coordinates
                    fprintf(out, "%d%s", rawVal, (x == (ot->W - 1)) ? "" : ","); //print it into the file
                }
                fprintf(out, "]%s\n", (y == (ot->L -1)) ? "" : ","); //These lines just add commas in the right places
            }
             fprintf(out, "]%s\n", (z == (ot->H -1)) ? "" : ",");
        }
        fprintf(out, "]%s\n",  (t == (ot->tf - 1)) ? "" : ",");
    }
    printf(" Done\n");
}

void extendTimeHorizon(ot_t *ot, int n) {
    //Extend time horizon by n timesteps
    long oldSize = (((ot->W * ot->L * ot->H * (ot->tf + 1)) + 3) / 4); //same formula as createTensor
    long newSize = (((ot->W * ot->L * ot->H * (ot->tf + n + 1)) + 3) / 4); //calculate new size with additional time steps

    uint8_t* newMap = realloc(ot->occupancymap, newSize); //call realloc to get more memory
    if (!newMap) { //memory allocation error check
        fprintf(stderr, "Failed to extend tensor memory :( \n");
        exit(EXIT_FAILURE); 
    }
    ot->occupancymap = newMap;  //update pointer

    //set everything everything in the newly allocated area to zeros (unoccupied)
    memset(ot->occupancymap + oldSize, 0, newSize - oldSize);
    ot->tf = ot->tf + n; //update the tensor time horizon metadata
}