/*

    4DMVP Project for Collins Aerospace
    Author: Nathan Collins
    Date Created: 8/18/24
    Last Updated: 11/24/24
*/

#include "pathfind.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>



const int pace = 3; //maximum random increase in path request time for each monte carlo sub-iteration

void monteCarloIteration(int charLength, int charHeight, int maxIter) {
    //function that runs a map until a max iteration count or failure
    //demonstrates how the algorithm is used:
    map_t *map = emptyMap(charLength, charLength, charHeight, 1); //allocate the map
    FILE* tensorOut = fopen("tensor.csv", "w"); //open a file to record the tensor after run is done
    FILE* shortPathInfo = fopen("paths.csv","w"); //open a file to record stats on each path 
    fprintf(shortPathInfo, "ID, IdealT0, H0, Gf, TTS (ms)\n");
    printf("Monte Carlo Iteration Start: (%dX%dX%d) \n", charLength, charLength, charHeight);
    vehicle_t vehicle = { //define a vehicle performance profile to use
        .W = 2280 * 9.81 * CHI / pow(TAU,2), //vehicle weight in canonical units
        //source: https://pdf.sciencedirectassets.com/271985/1-s2.0-S1270963822X00059/1-s2.0-S1270963822002358/main.pdf?X-Amz-Security-Token=IQoJb3JpZ2luX2VjEAMaCXVzLWVhc3QtMSJHMEUCIQCsnbzZXViA%2BwsebKIlFgax1klkCZz7IIPtQzgminbsogIgdLrpg7Iuz%2BsN66LdyeBiQVrEn7zlB1qaUn6UXkZnv54qsgUIHBAFGgwwNTkwMDM1NDY4NjUiDCYqKQKS%2BlPbrUHjNyqPBd9MnBZBRGw8vCQl5I%2BnQX0lkg5me6ZZybdmJSkuGzRFRDgltoTA7EEDE57SW6Jo1ZwZOaJPtN7FkcaO%2FFt%2B0NHiM1jK1iHiB4Svelq4g1wvMOWSLphEzEndcoH7Zedo81BpAn9Cj6r%2FaxvNNewaeq86fKewKoWpbU5iJ5HIdTRktTjVfxuRb2B0tNdxJMYoZUzZN7sa%2B4fzygd0NLOESPz5mlwY0vBBmHmm4bcvuxhXCpcjBe3DS%2FhxVqsAuG0QSiyetgbgNN9LEU3AYLiqj1kDb3lmaRyUXi7h1ldFIAn12MPqLFDu%2F7BsbCGZ0PllxzU%2FEc%2BWGNOwBEj3Dz%2BlM0czTtzXlpliagDVOzfTQV7GzeH6Hg02zAEtaDllDjejOHRAE17UowLrtDEvw3l1oKDHaOcX9WYF2UeegdzeNVTWrjm1UlmsILTPlJ4M8DmZp7uOOevPuq1xik9kJy90LPSSsIP%2BySYxzRJndAdFPNwcidKOgFkwm1IIoh77TuwkEgO8hu35uOU9J%2F99kwWXdRUfCIN%2Fx9KuaqtcuBYIUqQPXd11uoTCpwZRE5Hu1kHaNcWJ6KP3l2ZHFRHWRu18wdpR8KtDCLOx1inew%2FVSotR5fjOWsPXLfSpQD%2BDBv0s%2F2uR2pjJEqVc2zSdy5f9PApsw1cGBLvrOKGbzCT0P0SLHuiVF1tQI4ra8tnHKvIGByZ2YDSJxEbm4dpBY7%2FDCHLrkP8bpsT22cpx1qBOH%2Br7JdyGDmKNR4Uc314dRSMMeAIMzYUmZCGMqAuNtIjKWDJvLPs2VTZ5HbzTL2CkcdJegkL%2FPpXHseUt7BthWZS%2F4Md3%2BjOUeqDs67EaF0Iyj5i8IXRdGI9kwQvDEOy0ps5YwjZiEvQY6sQF%2FaNfasJIZJAPdIvfEZ44QxsgHfP2Z0ON3hFolGm%2FOqt4D7R3RDi1izlYDffLmBzFevi3%2Bgixit%2FRhNLR2BovmLHUPvYtsaEWOUYmw0%2FQ1sSy4kw51BFQ202uMVd0ivd2EVD7xPWuu1L0EiMdicB%2FkdNoeuo0PZYLjoR9KShg4H48SGQWrV317y%2BPH1CIK7K3p%2FALtFosVrQGYWHGsCDoW4upmVTjTXu7MNirf1jJujBw%3D&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Date=20250203T194236Z&X-Amz-SignedHeaders=host&X-Amz-Expires=300&X-Amz-Credential=ASIAQ3PHCVTYYS34LYI7%2F20250203%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Signature=95f4ec3b4534c3d5a9bae8ad81817028dfa1bbc8be4cc06bbfe28a265f792744&hash=1db73697aa17cc7b15201915b6110d3d3086c3efbb2dcfa9caeb9d93dcc2da8f&host=68042c943591013ac2b2430a89b270f6af2c76d8dfd086a07176afe7c76c2c61&pii=S1270963822002358&tid=spdf-145ddbfa-2327-4094-9d2e-ceb7ae245a10&sid=e49c6afa289f294e89894d284836f9fe98f8gxrqa&type=client&tsoh=d3d3LnNjaWVuY2VkaXJlY3QuY29t&ua=131d5c5301530b510055&rr=90c4e7b5ef672069&cc=us
        .LD = 12,
        .hoverPower = 2280 * 9.81 * CHI / pow(TAU,2) /2, //NOT REPRESENTATIVE
        .hdot = 1,
        .psidot = 0.1,
        .R = 2,
        .vdot = 0.05,
        .vpref = 2,
        .vstall = 1
    };
    int tau_randomizer = 0; //an initial "request time" counter used to space out path requests.
    for(int i = 0; i < maxIter; i++) { //loop until max iterations hit
        printf("Iteration #%d\n", i+1);
        srand(clock());
        path_t *path = (path_t*)calloc(1,sizeof(path_t));
        path->vehicle = vehicle;
        path->ideal_tau_start = tau_randomizer;
        tau_randomizer += rand() % pace; //increment the next path request time by a random number
        int z = 5; //all vehicles start/end at an identical height (as they would in real life)
        point_t start = {.x = rand() % charLength, .y = rand() % charLength, .z = z}; //create random start and end coordinates
        point_t end = {.x = rand() % charLength, .y = rand() % charLength, .z = z};
        path->startPoint = start; //add the start and end coordinates to the path
        path->endPoint = end;
        pathFind(map, path); //call the main pathfinding algorithm
        if(path->solved == -1) { //monte carlo exits at first failure - algorithm should not fail unless there's an error.
            printf("Path #%d failed to solve. Monte Carlo exiting.\n", path->id);
            //exportPathShort(shortPathInfo, path);
            //exportPathShort(statFile, path);
            freePath(path);
            break;
        }
        exportPathShort(shortPathInfo, path); //record stats on the solution
        //exportPathShort(statFile, path);
        exportPath(path); //export the actual path for future visualization with the matlab tool
        writePath(map, path); //write the solution into the tensor
        freePath(path); //free the path
        if(i == maxIter-1) { //debugging statement
            printf("Maximum Iteration count reached. Monte Carlo exiting.\n");
        }
    }
    resetGlobalIdCounter(); //resets the global id counter after each monte carlo iteration in case multiple are being run
    exportMap(map, tensorOut); //export the final tensor into the file
    fclose(shortPathInfo); //close the stats file
    fclose(tensorOut); //close the tensor export file
    freeMap(map); //free the map
    //clean exit - all memory freed.
}

int main() {
    monteCarloIteration(50, 20, 100);
}