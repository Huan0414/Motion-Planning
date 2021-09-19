#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include "node.h"

namespace navigation_astar
{
    class AstarPathFinder
    {
    protected:
        uint8_t *data;
        GridNodePtr **GridNodeMap;
        SIndex goalIdx;
        int GLX_SIZE, GLY_SIZE;
        int GLXY_SIZE;

        double resolution, inv_resolution;
        double gl_xl, gl_yl;
        double gl_xu, gl_yu;

        GridNodePtr terminatePtr;

        std::multimap<double, GridNodePtr> openSet;

        double getHeu(GridNodePtr node1, GridNodePtr node2);

        void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> &neighborPtrSets,
                          std::vector<double> &edgeCostSets);

        bool isOccupied(const int &idx_x, const int &idx_y) const;
        bool isOccupied(const SIndex &index) const;
        bool isFree(const int &idx_x, const int &idx_y) const;
        bool isFree(const SIndex &index) const;

        Points gridIndex2coord(const SIndex &index);
        SIndex coord2gridIndex(const Points &pt);


    public:
        AstarPathFinder(){};
        ~AstarPathFinder(){};
        void AstarGraphSearch(Points start_pt, Points end_pt);
        void resetGrid(GridNodePtr ptr);
        void resetUsedGrids();

        void initGridMap(double _resolution, Points global_xy_l, Points global_xy_u, 
                                                                                                 int max_x_id, int max_y_id);
        void setObs(const double coord_x, const double coord_y);

        Points coordRounding(const Points &coord);
        std::vector<Points> getPath();
        std::vector<Points> getVisitedNodes();
    };
}

#endif