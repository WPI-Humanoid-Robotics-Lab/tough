#ifndef ROVER_DETECTION_H
#define ROVER_DETECTION_H

#define MAP_RESOLUTION 0.05
#define MAP_HEIGHT     50/MAP_RESOLUTION
#define MAP_WIDTH      50/MAP_RESOLUTION
#define MAP_X_OFFSET   -5.0
#define MAP_Y_OFFSET   -25.0

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <val_common/val_common_names.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class PathSubtraction{
public:
    PathSubtraction(ros::NodeHandle &nh);
    ~PathSubtraction();
    void getProjMap(const nav_msgs::OccupancyGrid projMap);
    void getPathMap(const nav_msgs::OccupancyGrid pathMap);
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber projMap_sub_;
    ros::Publisher  roverMap_pub_;
    nav_msgs::OccupancyGrid occGrid_proj;
    nav_msgs::OccupancyGrid occGrid_map;
    nav_msgs::OccupancyGrid occGrid_rover;
    size_t getIndex1(int x, int y);
    size_t getIndex2(int x, int y);
};

#endif // ROVER_DETECTION_H
