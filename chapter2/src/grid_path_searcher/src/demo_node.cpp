#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "Astar_searcher.h"
#include "JPS_searcher.h"
#include "backward.hpp"

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

// launch 文件的仿真参数
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    

// 全局变量
bool _has_map   = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ros 通信的订阅者、发布者
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher  _grid_path_vis_pub, _visited_nodes_vis_pub, _grid_map_vis_pub;

AstarPathFinder * _astar_path_finder     = new AstarPathFinder();
JPSPathFinder   * _jps_path_finder       = new JPSPathFinder();

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);

void visGridPath( vector<Vector3d> nodes, bool is_use_jps );
void visVisitedNode( vector<Vector3d> nodes );
void pathFinding(const Vector3d start_pt, const Vector3d target_pt);

// 订阅终点坐标信息的回调函数
void rcvWaypointsCallback(const nav_msgs::Path & wp)
{
    // 终点在地面下或没有地图则无法进行路径规划     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Vector3d target_pt;
    // 获取交互界面给出的终点坐标
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    // 输入起点、终点，调用 pathFinding() 进行路径规划
    pathFinding(_start_pt, target_pt); 
}

// 订阅地图点云信息的回调函数
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    // 有地图点云信息，则直接返回
    if(_has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    // 将 ROS 点云消息转换为 PCL 点云数据结构
    pcl::fromROSMsg(pointcloud_map, cloud);
    
    // 判断点云是否为空
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    // 遍历点云数据结构
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {
        pt = cloud.points[idx];        

        // 将障碍物信息设置到栅格地图中，为路径规划做准备
        _astar_path_finder->setObs(pt.x, pt.y, pt.z);
        _jps_path_finder->setObs(pt.x, pt.y, pt.z);

        // for visualize only
        // 将点云实际坐标转换成对应栅格中心点的实际坐标
        Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    // 在无组织的点云数据集中指定点云的总点数
    cloud_vis.width    = cloud_vis.points.size();
    // 在无组织的点云数据集中设置为1
    cloud_vis.height   = 1;
    // 判断点云中的点是否包含 Inf/NaN 这种值(包含为 false)
    cloud_vis.is_dense = true;

    // 将 PCL 点云数据结构转换为 ROS 点云消息
    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    // 地图点云可视化
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

// 调用路径规划算法
void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{
    // 使用 A* 进行路径规划
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);

    // 获取规划的路径
    auto grid_path     = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    // 可视化结果
    visGridPath (grid_path, false);
    visVisitedNode(visited_nodes);

    // 重置地图，为下一次调用做准备
    _astar_path_finder->resetUsedGrids();

    //_use_jps = 0 -> Do not use JPS
    //_use_jps = 1 -> Use JPS
    //you just need to change the #define value of _use_jps
    // 进行 JPS 路径规划编写时，将 _use_jps 置为 1
#define _use_jps 0
#if _use_jps
    {
        //Call JPS to search for a path
        _jps_path_finder -> JPSGraphSearch(start_pt, target_pt);

        //Retrieve the path
        auto grid_path     = _jps_path_finder->getPath();
        auto visited_nodes = _jps_path_finder->getVisitedNodes();

        //Visualize the result
        visGridPath   (grid_path, _use_jps);
        visVisitedNode(visited_nodes);

        //Reset map for next call
        _jps_path_finder->resetUsedGrids();
    }
#endif
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    // 订阅地图点云信息的回调函数
    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    // 订阅终点坐标信息的回调函数
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );

    // 地图可视化
    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    // 路径规划可视化
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    // 所有扩展节点可视化
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);

    // 点云间隙
    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    // 栅格地图分辨率，即栅格地图中一个栅格代表的实际长度
    nh.param("map/resolution",    _resolution,   0.2);
    
    // 实际地图大小(m)
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );
    
    // 起点坐标
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    // 实际地图坐标的下界、上界
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    // 地图分辨率的倒数，
    _inv_resolution = 1.0 / _resolution;
    
    // 栅格地图坐标最大值
    // 实际地图上 x,y 坐标有正负，但栅格地图中坐标是从 0 开始到最大索引值
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    // 定义了结构体 AstarPathFinder 变量_astar_path_finder，该结构体存储、实现了 Astar 路径规划所需的所有信息和功能
    _astar_path_finder  = new AstarPathFinder();
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    //定义了结构体 JPSPathFinder 变量_jps_path_finder，该结构体存储、实现了 JPS 路径规划所需的所有信息和功能
    _jps_path_finder    = new JPSPathFinder();
    _jps_path_finder    -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    delete _astar_path_finder;
    delete _jps_path_finder;
    return 0;
}

// 可视化路径
void visGridPath( vector<Vector3d> nodes, bool is_use_jps )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    
    if(is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    // marker 类型
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    // marker 朝向
    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    // marker 颜色
    if(is_use_jps){
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }

    // marker 大小
    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        // marker 点的坐标，geometry_msgs::Point 类型
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);
        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

// 可视化扩展过的节点
void visVisitedNode( vector<Vector3d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}