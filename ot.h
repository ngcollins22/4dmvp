#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
    Header file containing data structures of maps and occupancy tensors
*/


/*
    A struct that contains the large array representing the space occupancy. 
    Metadata includes only the spatial and temporal dimensions.
*/
typedef struct OccupancyTensor {
    uint8_t* occupancymap; //data representation of the occupancy, compressed to avoid memory problems
    int W, L, H; //dimensions of the physical space
    int tf; //dimensions of the time space
    /*
        like [x][y][z][t] -> 0 = not occupied, not close to anything that is
                             1 = within the actual geofence of a vehicle
                             2 = close enough to a geofence to warrant a detailed spacecheck
        Because each voxel is represented by only 2 bits, they are stored in blocks of 4 inside 8-bit integers.
        As a result, there are two types of indices: 8-bit indices (containing 4 voxels each)
                                                     and 2-bit indices (the index of an actual voxel)
                                                     a number of bit operations within setValueAt and getValueAt handle the compression
    */
} ot_t;

/*
    A map has dimensions, an occupancy tensor, and a few other pieces of metadata
*/
typedef struct Map {
    int W, L, H; //Dimensions of the map, in chi units

    //these times not used currently by algorithm, but likely useful for implementation.
    time_t referenceTime; // UTC "start of day"
    time_t startTime; // UTC start of this given map (should typically be now)

    int tau; // integer number of seconds between timesteps
    int chi; // integer number of meters as length of cube space step
    /*
        Tau and Chi are really not in the algorithm itself, but form the base units of the space.
        Not that vehicle performance profiles and all time values found in other states are in these units!
    */
    ot_t *occupancytensor; // 4D tensor representing occupancy over time
} map_t;

/*
    Create a new blank map with given spatial dimensions.
    Assumed reference time to be zero utc, same with startTime

    @param w width in chi units (x dimension)
    @param l length in chi units (y dimension)
    @param h height in chi units (z dimension)
    @param tf final time to initialize to. Should not be zero. Typically 1.

    @return initialized map.
*/
extern map_t *emptyMap(int w, int l, int h, int tf);

/*
    Pull the value of a map at some integer x,y,z,t coordinates.

    @param map Map to query
    @param x X-coordinate
    @param y Y-coordinate
    @param x Z-coordinate
    @param t T-coordinate
    
    @return value of voxel as described in occupancy tensor description. -1 if the index does not exist.
*/
extern int pull(map_t *map, int x, int y, int z, int t);

/*
    Creates a new tensor with given dimensions

    @param w integer width
    @param l integer length
    @param h integer height
    @param tf integer final time

    @return pointer to newly created tensor
*/
extern ot_t *createTensor(int w, int l, int h, int tf);

/*
    Frees a map, and the tensor inside of it. Called during wrap-up.

    @param map Pointer to map to free
*/
extern void freeMap(map_t *map);

/*
    Frees a tensor and the array within it.

    @param ot Pointer to tensor to free.
*/
extern void freeTensor(ot_t *ot);

/*
    Takes spatial and temporal coordinates and returns the corresponding two-bit index. 
    Undefined behavior if x/y/z/t coordinate exceed bounds

    @param ot Occupancy tensor to query
    @param x X-coordinate
    @param y Y-coordinate
    @param z Z-coordinate
    @param t T-coordinate.

    @return Index of two-bit value. NOT index of 8 bit value within tensor's array.
*/
extern int getIndex(ot_t *ot, int x, int y, int z, int t);

/*
    Gets the two-bit value at the two-bit index provided. Finds the associated 8-bit value and pulls out the 2-bit value.

    @param ot The occupancy tensor to query
    @param index The two-bit index (NOT the 8-bit index of the array in the tensor)

    @return two-bit value at the provided index.
*/
extern int getValueAt(ot_t *ot, int index);

/*
    Sets the value in the tensor at the two-bit index to the value provided.
    Value must be a 0,1,2 as described above. Not safe to other values.

    @param ot Occupancy tensor to update
    @param index Two-bit index
    @param value Two-bit value to set
*/
extern void setValueAt(ot_t *ot, int index, uint8_t value);

/*
    Exports the entire map and most of its metadata to the provided file. Should be an empty file.

    @param map The map to export
    @param out File pointer to write to
*/
extern void exportMap(map_t *map, FILE* out);


/*
    Utility function to extend the length of a tensor's array to a certain new time horizon.
    Occasionally fails on memory-constrained systems. This function is generally called when a new path
    is written into map, and the path's final time extends beyond the current time horizon of the tensor.

    @param ot The tensor to extend
    @param n The number of integer time steps to extend the time horizon by.
*/
extern void extendTimeHorizon(ot_t *ot, int n);

/*
    Push a new two-bit value into a map. Has internal priority rules to ensure other geofences are not overwritten.

    @param map The map to update
    @param x X-coordinate
    @param y Y-coordinate
    @param z Z-coordinate
    @param t T-coordinate
    @param val Two-bit value to push into the location. Must be 1 or 2. A 2 will not overwrite a 1 because a 1 is a higher priority value.
*/
extern void push(map_t *map, int x, int y, int z, int t, int val);