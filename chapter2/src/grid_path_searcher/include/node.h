#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
    // 1--> open set, -1 --> closed set     
    int id;        
    // 实际地图坐标
    Eigen::Vector3d coord; 
    // 扩展方向(JPS需要)
    Eigen::Vector3i dir;
    // 栅格地图坐标(索引)
    Eigen::Vector3i index;
	
    // g(n) 和 f(n)
    double gScore, fScore;
    // 父节点
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		id = 0;
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();

		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};


#endif
