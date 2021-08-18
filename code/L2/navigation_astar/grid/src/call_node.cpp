#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

#include "Astar_searcher.h"
//#include "Jps_searcher.h"

using namespace std;
using namespace Eigen;
using namespace navigation_astar;

ofstream pose_road_points;

double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size;
double start_pose_x, start_pose_y;
bool _has_map = false;

Points _start_pt;

Points _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

ros::Subscriber _map_sub, _pts_sub;
ros::Publisher _grid_path_vis_pub, _visited_nodes_vis_pub, _grid_map_vis_pub;

AstarPathFinder *_astar_path_finder = new AstarPathFinder();
//JPSPathFinder *_jps_path_finder = new JPSPathFinder();

void rcvWaypointsCallback(const nav_msgs::Path &wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);

void visGridPath(vector<Points> nodes, bool is_use_jps);
void visVisitedNode(vector<Points> nodes);
void pathFinding(const Points start_pt, const Points target_pt);

void rcvWaypointsCallback(const nav_msgs::Path &wp)
{
    if (wp.poses[0].pose.position.z < 0.0 || _has_map == false)
        return;
    //给出的终点坐标
    Points target_pt;
    target_pt.x = wp.poses[0].pose.position.x;
    target_pt.y = wp.poses[0].pose.position.y;

    ROS_INFO("[node] receive the planning target");

    pathFinding(_start_pt, target_pt);
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
    // if (_has_map)
    //     return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);
    if ((int)cloud.points.size() == 0)
        return;

    //加一个pt_inf改变栅格状态
    pcl::PointXYZ pt, pt_inf;

    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {
        pt = cloud.points[idx];

        if (pose_road_points.fail())
        {
            ROS_INFO("erro open the file");
            cout << "erro open the file" << endl;
            exit(0);
        }
        pose_road_points << pt.x << ", " << pt.y << ", " << pt.z << endl;

        //将障碍物信息设置进入栅格化地图,为后续路径规划做准备
        _astar_path_finder->setObs(pt.x, pt.y);
        //_jps_path_finder->setObs(pt.x, pt.y);

        Points apt;
        apt.x = pt.x;
        apt.y = pt.y;

        Points cor_round = _astar_path_finder->coordRounding(apt);

        //  2D栅格图
        pt_inf.x = cor_round.x;
        pt_inf.y = cor_round.y;

        cloud_vis.points.push_back(pt_inf);
    }

    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1;
    cloud_vis.is_dense = true;
    pcl::toROSMsg(cloud_vis, map_vis);

    // 可视化地图部分
    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

void pathFinding(const Points start_pt, const Points target_pt)
{
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);

    //获取规划的路径
    auto grid_path = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    //为下次规划重置地图
    _astar_path_finder->resetUsedGrids();

    //可视化结果
    visGridPath(grid_path, false);
    visVisitedNode(visited_nodes);

#define _use_jps 0
#if _use_jps
    {
        _jps_path_finder->JPSGraphSearch(start_pt, target_pt);
        auto grid_path = _jps_path_finder->getPath();
        auto visited_nodes = _jps_path_finder->getVisitedNodes();

        //重置下次的地图
        _jps_path_finder->resetUsedGrids();

        //将结果可视化
        visGridPath(grid_path, _use_jps);
        visVisitedNode(visited_nodes);
    }
#endif
}

//将结果可视化
void visGridPath(vector<Points> nodes, bool is_use_jps)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";

    if (is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if (is_use_jps)
    {
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else
    {
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Points coord = nodes[i];
        pt.x = coord.x;
        pt.y = coord.y;

        node_vis.points.push_back(pt);
    }
    _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode(vector<Points> nodes)
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
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Points coord = nodes[i];
        pt.x = coord.x;
        pt.y = coord.y;

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}

int main(int argc, char **argv)
{

    pose_road_points.open("/home/spiderman/akewoshi/src/navigation_astar/pose.txt");
    pose_road_points.setf(ios::fixed);
    pose_road_points.setf(ios::showpoint);
    pose_road_points.precision(2); //显示两位小数点

    ros::init(argc, argv, "call_node");
    ros::NodeHandle nh("~"); //用波浪号（~）字符将其声明为私有，则话题名称将变为/node1/bar。

    _map_sub = nh.subscribe("map", 1, rcvPointCloudCallBack);
    _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallback);

    _grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _visited_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis", 1);

    nh.param("map/cloud_margin", _cloud_margin, 0.0);
    nh.param("map/resolution", _resolution, 0.2);

    nh.param("map/x_size", _x_size, 50.0);     //单位是 m
    nh.param("map/y_size", _y_size, 50.0);

    nh.param("planning/start_x", start_pose_x, 0.0); 
    nh.param("planning/start_y", start_pose_y, 0.0);

    _start_pt.x = start_pose_x;
    _start_pt.y = start_pose_y;

    _map_lower.x = -_x_size / 2.0;  //-30
    _map_lower.y = -_y_size / 2.0;  //-30
    _map_upper.x = _x_size / 2.0;   //30
    _map_upper.y = _y_size / 2.0;  //30

    _inv_resolution = 1.0 / _resolution; //10

    // 地图x,y最大index
    _max_x_id = (int)(_x_size * _inv_resolution);  //60 * 10 = 600
    _max_y_id = (int)(_y_size * _inv_resolution);  //60 * 10 = 600

    _astar_path_finder = new AstarPathFinder();
    //(10,  (-30,  -30, ), (30, 30), 600, 600)
    _astar_path_finder->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id);

   // _jps_path_finder = new JPSPathFinder();
    //_jps_path_finder->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id);

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    delete _astar_path_finder;
    //delete _jps_path_finder;

    pose_road_points.close();
    return 0;
}
