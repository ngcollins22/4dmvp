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

int main() {
    //runTest(1000, 100, 0);
    //runTest(10, 1000, 0.1);
    //runTest(10, 1000, 0.2);
    //runTest(10, 1000, 0.3);
    //runTest(10, 1000, 0.4);
    //runTest(10, 1000, 0.5);
    //runTest(10, 1000, 0.6);
    //runTest(10, 1000, 0.7);
    //runTest(10, 1000, 0.8);
    //runTest(10, 1000, 0.9);


    FILE* out = fopen("test.csv", "w");
    runIteration(out, 100, 0.3, 1);
    fclose(out);
}