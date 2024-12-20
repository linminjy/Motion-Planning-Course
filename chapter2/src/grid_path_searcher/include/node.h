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
    // 1--> open set(待访问), -1 --> closed set(已访问)
    // 0--> 一般节点(未访问)，可能是占据节点或自由节点
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
    // 保存 openSet 中的指向该节点的迭代器(不能使用！！！)，但是插入、删除 openSet 中的节点会导致该迭代器失效
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
