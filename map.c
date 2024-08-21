
#include "map.h"

/*
    Returns a pointer to a blank map with given length and width
*/
map_t *create_blank_map(unsigned int l, unsigned int w) {
    map_t* map = (map_t*) malloc(sizeof(map_t));
    map->l = l;
    map->w = w;
    map->grid = (node_t *) malloc(sizeof(node_t)*l*w);
    return map;
}

/*
    Populates a map with a number of obstacles
*/
extern void populate_with_obstacles(map_t* map, unsigned int numObstacles);

/*
    Get a node at a specific index. Returns null if index invalid
*/
extern node_t *getIndex(map_t* map, int n);

/* 
    Get a node at a specific x and y location. Returns null if index invalid
*/
extern node_t *getLocation(map_t* map, int x, int y);

