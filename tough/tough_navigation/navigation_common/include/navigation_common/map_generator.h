#ifndef MAP_GENERATOR_H
#define MAP_GENERATOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "tough_controller_interface/robot_state.h"
#include "tough_common/robot_description.h"
#include <mutex>


enum CELL_STATUS{
    FREE = 0,
    VISITED = 50,
    BLOCKED = 90,
    OCCUPIED = 100
};

class MapGenerator{

public:
    MapGenerator(ros::NodeHandle &n);
    ~MapGenerator();

    static void trimTo2DecimalPlaces(float &x, float &y);
    static size_t getIndex(float x, float y);

private:

    static const float MAP_RESOLUTION;
    static const float MAP_HEIGHT;
    static const float MAP_WIDTH;
    static const float MAP_X_OFFSET;
    static const float MAP_Y_OFFSET;

    std::mutex mtx;

    void resetMap(const std_msgs::Empty &msg);
    void clearCurrentPoseCB(const std_msgs::Empty &msg);
    void convertToOccupancyGrid(const sensor_msgs::PointCloud2Ptr msg);
    void updatePointsToBlock(const sensor_msgs::PointCloud2Ptr msg);
    void timerCallback(const ros::TimerEvent& e);

    ros::NodeHandle nh_;
    ros::Subscriber pointcloudSub_;
    ros::Subscriber resetMapSub_;
    ros::Subscriber clearCurrentPoseSub_;
    ros::Subscriber blockMapSub_;
    ros::Publisher  mapPub_;
    ros::Publisher  visitedMapPub_;
    nav_msgs::OccupancyGrid occGrid_;
    nav_msgs::OccupancyGrid visitedOccGrid_;
    sensor_msgs::PointCloud2 pointsToBlock_;
    ros::Timer timer_;
    RobotStateInformer* currentState_;
    RobotDescription* rd_;
};

#endif // MAP_GENERATOR_H


