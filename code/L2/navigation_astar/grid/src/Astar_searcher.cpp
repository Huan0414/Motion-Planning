#include "Astar_searcher.h"
#include <chrono>

using namespace std;
using namespace navigation_astar;

bool tie_break = false;
//(10,  (-30,  -30, ), (30, 30), 600, 600)
void AstarPathFinder::initGridMap(double _resolution, Points global_xy_l, Points global_xy_u,
                                  int max_x_id, int max_y_id)
{
    gl_xl = global_xy_l.x;
    gl_yl = global_xy_l.y;

    gl_xu = global_xy_u.x;
    gl_yu = global_xy_u.y;

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLXY_SIZE = GLX_SIZE * GLY_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data = new uint8_t[GLXY_SIZE];
    memset(data, 0, GLXY_SIZE * sizeof(uint8_t));

    GridNodeMap = new GridNodePtr *[GLX_SIZE];      //初始化地图的行数

    for (int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr [GLY_SIZE];  //初始化地图的列数
        for (int j = 0; j < GLY_SIZE; j++){
            SIndex tmpIdx;
            tmpIdx.xs = i;
            tmpIdx.ys = j;
            Points pos = gridIndex2coord(tmpIdx);
            GridNodeMap[i][j] = new GridNode(tmpIdx, pos);
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; i++)
    {
        for (int j = 0; j < GLY_SIZE; j++)
        {
            resetGrid(GridNodeMap[i][j]);
        }
    }
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y)
{
    if(coord_x<gl_xl || coord_y < gl_yl || coord_x >= gl_xu || coord_y >= gl_yu)
    {
        return;
    }
    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
    data[(idx_x * GLY_SIZE) +  idx_y] = 1;
}

vector<Points> AstarPathFinder::getVisitedNodes()
{
    vector<Points> visited_nodes;
    for (int i = 0; i < GLX_SIZE; i++)
    {
        for (int j = 0; j < GLY_SIZE; j++)
        {
            if(GridNodeMap[i][j]->id == -1)
            {
                visited_nodes.push_back(GridNodeMap[i][j]->coord);
            }
        }
    }
    ROS_WARN("visited_nodes size: %zd", visited_nodes.size());
    return visited_nodes;
}

//index ---> coord
Points AstarPathFinder::gridIndex2coord(const SIndex &index)
{
    Points pt;
    pt.x = ((double)index.xs + 0.5) * resolution + gl_xl;
    pt.y = ((double)index.ys + 0.5) * resolution + gl_yl;

    return pt;
}

//coord ---> index
SIndex AstarPathFinder::coord2gridIndex(const Points &pt)
{
    SIndex idx;
    idx.xs = min(max(int((pt.x - gl_xl) * inv_resolution), 0), GLX_SIZE - 1);
    idx.ys = min(max(int((pt.y - gl_yl) * inv_resolution), 0), GLY_SIZE - 1);
    return idx;
}

Points AstarPathFinder::coordRounding(const Points &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const SIndex &index) const
{
    return isOccupied(index.xs, index.ys);
}

inline bool AstarPathFinder::isFree(const SIndex &index) const
{
    return isFree(index.xs, index.ys);
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
            (data[(idx_x * GLY_SIZE) + idx_y] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
            (data[(idx_x * GLY_SIZE) + idx_y] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets,
                                        vector<double> &edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
    if(currentPtr == nullptr)
    {
        std::cout << "Erro: nothing in open list" << endl;
        //return;
    }

    SIndex thisNode = currentPtr->index;
    int this_x = thisNode.xs;
    int this_y = thisNode.ys;

    auto this_coord = currentPtr -> coord;

    int n_x, n_y;
    double dist;
    GridNodePtr temp_ptr = nullptr;
    Points n_coord;

    for (int i = -1; i <= 1; ++i)
    {
        for (int j = -1; j <= 1; ++j)
        {
            if(i == 0 && j == 0){
                continue;
            }
            n_x = this_x + i;
            n_y = this_y + j;

            if((n_x < 0) || (n_x > (GLX_SIZE - 1)) || (n_y < 0) || (n_y > (GLY_SIZE - 1)))
            {
                continue;
            }

            if(isOccupied(n_x, n_y))
            {
                continue;
            }

            temp_ptr = GridNodeMap[n_x][n_y];
            if(temp_ptr->id == -1)
            {
                continue;
            }
            n_coord = temp_ptr->coord;

            if(temp_ptr == currentPtr)
            {
                std::cout << "Erro: temp_ptr == currentPtr" << std::endl;
                return;
            }
            
            if((std::abs(n_coord.x - this_coord.x) < 1e-6) and (std::abs(n_coord.y - this_coord.y) < 1e-6))
            {
                std::cout << "Error. Not expanding correctly!" << std::endl;
                std::cout << "n_coord: " << n_coord.x << " " << n_coord.y << std::endl;
                std::cout << "this_coord: " << this_coord.x << " " << this_coord.y << std::endl;

                std::cout << "current node index: " << this_x << " " << this_y << " "  << std::endl;
                std::cout << "neighbor node index: " << n_x << " " << n_y << " "  << std::endl;
            }

            dist = std::sqrt((n_coord.x - this_coord.x) * (n_coord.x - this_coord.x) + (n_coord.y - this_coord.y) * (n_coord.y - this_coord.y));
            neighborPtrSets.push_back(temp_ptr);
            edgeCostSets.push_back(dist);
        }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    double h;
    auto node1_coord = node1->coord;
    auto node2_coord = node2->coord;

    //H >>> 曼哈顿距离
    h = std::abs(node1_coord.x - node2_coord.x) + std::abs(node1_coord.y - node2_coord.y);

    //H >>> 欧几里德距离
    h = std::sqrt(std::pow((node1_coord.x - node2_coord.y), 2) + std::pow((node1_coord.x - node2_coord.y), 2));

    //H >>> 对角线距离  closed-form solution
    double dx = std::abs(node1_coord.x - node2_coord.x);
    double dy = std::abs(node1_coord.y - node2_coord.y);
    double min_xyz = std::min({dx, dy});
    h = dx + dy + (std::sqrt(2.0) - 2) * min_xyz;

    if (tie_break)
    {
        double p = 1.0 / 25.0;
        h = h * (1.0 + p);
    }

    return h;
}

void AstarPathFinder::AstarGraphSearch(Points start_pt, Points end_pt)
{
    ros::Time time_1 = ros::Time::now();

    SIndex start_idx = coord2gridIndex(start_pt);
    SIndex end_idx = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    openSet.clear();

    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    //放入openSet中, id == 1
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr, endPtr);
    startPtr -> id = 1;
    startPtr -> coord = start_pt;
    openSet.insert(make_pair(startPtr->fScore, startPtr));
    GridNodeMap[start_idx.xs][start_idx.ys]->id = 1;

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    //main loop
    while(!openSet.empty())
    {
        int x = openSet.begin()->second->index.xs;
        int y = openSet.begin()->second->index.ys;
        openSet.erase(openSet.begin());
        currentPtr = GridNodeMap[x][y];

        if(currentPtr->id == -1){
            continue;
        }
        currentPtr->id = -1;

        if (currentPtr->index.xs == goalIdx.xs && currentPtr->index.ys == goalIdx.ys)
        {
            //chrono::steady_clock::time_point time_2 = chrono::steady_clock::now();
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution);
            return;
        }

        //未被扩展的结点需要扩展，获取邻居结点和距离
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        for(int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            //根据 id 值判断邻居是否可以扩展
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == 0)
            {
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                neighborPtr->cameFrom = currentPtr;
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                neighborPtr->id = 1;
                continue;
            }
            else if(neighborPtr->id == 1)
            {
                if (neighborPtr->gScore > (currentPtr->gScore + edgeCostSets[i]))
                {
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr->cameFrom = currentPtr;
                }
                continue;
            }
            else
            {
                continue;
            }
        }
    }
    ros::Time time_2 = ros::Time::now();
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec());
}

//获得A*搜索的完整路径
vector<Points> AstarPathFinder::getPath()
{
    vector<Points> path;
    vector<GridNodePtr> gridPath;
    auto ptr = terminatePtr;
    while (ptr->cameFrom != NULL)
    {
        gridPath.push_back(ptr);
        ptr = ptr->cameFrom;
    }

    for (auto ptr: gridPath){
        path.push_back(ptr->coord);
    }
    reverse(path.begin(), path.end());

    return path;
}
