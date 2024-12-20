#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

// 初始化栅格地图
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    // 实际地图坐标的下界
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    // 实际地图坐标的上界
    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    // 栅格地图栅格数量
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    // 栅格地图分辨率
    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    // 将三维数组拉伸成一维数组，每个元素表示栅格的占据状态
    // 指针 data 指向动态分配的一维数组
    data = new uint8_t[GLXYZ_SIZE];
    // 将一维数组每个元素初始化为 0
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    // 初始化栅格地图中每个栅格节点
    // 指针 GridNodeMap 指向动态分配的 GridNodePtr ** 数组
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        // GridNodePtr ** 数组的元素 GridNodeMap[i] 指向动态分配的 GridNodePtr * 数组
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            // GridNodePtr * 数组的元素 GridNodeMap[i][j] 指向动态分配的 GridNodePtr 数组
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                // 栅格在栅格地图中的坐标
                Vector3i tmpIdx(i,j,k);
                // 栅格在实际地图中的坐标
                Vector3d pos = gridIndex2coord(tmpIdx);
                // GridNodePtr 数组的元素 GridNodeMap[i][j][k] 还是指针，指向动态分配的 GridNode 结构体
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

// 将节点设置成未访问状态
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

// 将栅格地图所有节点设置成未访问状态
void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

// 根据实际地图中障碍物的坐标设置栅格地图的占据状态
void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    // 障碍物坐标是否超出实际地图坐标的上界、下界
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    // 将障碍物坐标转换为栅格地图坐标
    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    // 找到一维数组中的线性索引，赋值为 1，表示该栅格是占据状态
    // 三维数组按照访问顺序拉伸成一维数组
    // 三维数组的 0、1、2 轴的对应的是栅格地图的 x、y、z 轴
    // 访问顺序：0 -> 1 -> 2 (x -> y -> z)，即先确定 x 的索引，再确定 y，最后是 z
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

// 获取所有访问过的节点
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

// 将栅格坐标 gridIndex 转换为实际坐标 coord
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    // 加上 0.5 使得栅格的实际坐标对应栅格中心点
    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

// 将实际坐标 coord 转换为栅格坐标 gridIndex
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    // 实际坐标可能在实际地图外
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

// 实际坐标近似，将实际坐标转化为对应栅格中心点的实际坐标
Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

// 判断节点是否为占据节点
inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

// 判断节点是否为自由节点
inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

// 判断节点是否为占据节点
inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

// 判断节点是否为自由节点
inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

// 返回当前节点的所有相邻节点，以及到相邻节点的代价函数值
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */

   Vector3i currentIndex = currentPtr->index;
    for (int dx = -1; dx < 2; ++dx)
        for (int dy = -1; dy < 2; ++dy)
            for (int dz = -1; dz < 2; ++dz)
            {
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;
                Vector3i neighborIndex = currentIndex + Vector3i(dx, dy, dz);
                // 相邻节点需要保证在栅格地图中
                if (neighborIndex(0) < 0 || neighborIndex(0) >= GLX_SIZE || 
                    neighborIndex(1) < 0 || neighborIndex(1) >= GLY_SIZE || 
                    neighborIndex(2) < 0 || neighborIndex(2) >= GLZ_SIZE)
                    continue;
                neighborPtrSets.push_back(GridNodeMap[neighborIndex(0)][neighborIndex(1)][neighborIndex(2)]);
                edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
            }
}

// 获取启发函数值(曼哈顿距离/欧式距离/对角线距离/0(Dijkstra))
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
   double tie_breaker = 1 + 1 / 1000;
   return tie_breaker * getDiagHeu(node1, node2);   
}

// 欧式距离启发函数
inline double AstarPathFinder::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    int dx = abs(node1->index(0) - node2->index(0));
    int dy = abs(node1->index(1) - node2->index(1));
    int dz = abs(node1->index(2) - node2->index(2));
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// 对角线距离启发函数
inline double AstarPathFinder::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    int dx = abs(node1->index(0) - node2->index(0));
    int dy = abs(node1->index(1) - node2->index(1));
    int dz = abs(node1->index(2) - node2->index(2));

    int min_id = min(min(dx, dy), dz);
    dx -= min_id;
    dy -= min_id;
    dz -= min_id;

    if dx == 0:
        return sqrt(3) * min_id + dy + dz + (sqrt(2) - 2) * min(dy, dz);
    if dy == 0:
        return sqrt(3) * min_id + dx + dz + (sqrt(2) - 2) * min(dx, dz);
    if dz == 0:
        return sqrt(3) * min_id + dx + dy + (sqrt(2) - 2) * min(dx, dy);
    return h;
}

// 曼哈顿距离启发函数
inline double AstarPathFinder::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    int dx = abs(node1->index(0) - node2->index(0));
    int dy = abs(node1->index(1) - node2->index(1));
    int dz = abs(node1->index(2) - node2->index(2));
    return dx + dy + dz;
}

// A* 路径规划算法
void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    // 获取起点和终点的栅格坐标
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    // 目标的栅格坐标
    goalIdx = end_idx;

    // 起点和终点的栅格中心点的实际坐标
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    // 初始化起始节点和终止节点
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    // openSet 是通过 STL 库中的 multimap 实现的 open list
    openSet.clear();
    // currentPtr 指向 open_list 中 f(n) 最小的节点
    GridNodePtr currentPtr  = NULL;
    // neighborPtr 指向当前节点的相邻节点
    GridNodePtr neighborPtr = NULL;

    startPtr -> gScore = 0;
    // f = h + g = h + 0
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    // 将起始节点标记为待访问
    startPtr -> id = 1; 
    // 将起始节点加入 openSet
    openSet.insert(make_pair(startPtr -> fScore, startPtr));
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */

       // multimap 使用默认比较函数是从小到大排序，所以取第一个即可得到最小 fScore 的节点
        auto it = openSet.begin();
        currentPtr = it->second;
        // 如果当前节点是目标节点，则结束搜索
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            // 目标节点 = 当前节点
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }

        // openSet 删除当前节点
        open_set.erase(it);
        // 将当前节点标记为已访问
        currentPtr->id = -1;
        
        // 获得当前节点的所有相邻节点及到相邻节点的代价函数值
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open list
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close list
            *        
            */
            neighborPtr = neighborPtrSets[i];
            // 占据节点或已访问节点则跳过
            if(isOccupied(neighborPtr->index) || neighborPtr->id == -1)
                continue;

            // 从起始节点经过当前节点到达相邻节点的代价
            double neighbor_gScore = currentPtr->gScore + edgeCostSets[i];
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                
                neighborPtr->id = 1;
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore = neighbor_gScore;
                neighborPtr->fScore = neighbor_gScore + getHeu(neighborPtr, endPtr);
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                continue;
            }
            else if(neighborPtr->gScore > neighbor_gScore){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                // 以空间换时间，直接生成具有更小 fScore 的节点加入 openSet
                // auto range = openSet.equal_range(neighborPtr->fScore);
                // for(auto it = range.first; it != range.second; ++it)
                // {
                //     Vector3i index = it->second->index;
                //     if(index == neighborPtr->index)
                //     {
                //         openSet.erase(it);
                //         break;
                //     }
                // }
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore = neighbor_gScore;
                neighborPtr->fScore = neighbor_gScore + getHeu(neighborPtr, endPtr);                
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}

// 获取 A* 搜索到的完整路径
vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    for (auto ptr = terminatePtr; ptr != NULL; ptr = ptr->cameFrom)
        gridPath.push_back(ptr);
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}