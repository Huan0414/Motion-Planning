#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <map>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>

namespace navigation_astar
{
    #define inf 1 >> 20
    struct GridNode;
    typedef GridNode *GridNodePtr;

    typedef struct 
    {
        float x;
        float y;
    } Points;

    typedef struct
    {
        int xs;
        int ys;
    } SIndex;

    struct GridNode
    {
        int id;
        Points coord;
        SIndex dir;
        SIndex index;
        double gScore, fScore;

        GridNodePtr cameFrom;
        std::multimap<double, GridNodePtr>::iterator nodeMaplt;

        GridNode(SIndex &_index, Points &_coord)
        {
            id = 0;
            index = _index;
            coord = _coord;

            dir.xs = inf;
            gScore = inf;
            fScore = inf;
            cameFrom = NULL;
        }
        GridNode(){};
        ~GridNode(){};
    };
}

#endif
