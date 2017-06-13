# pragma once

#include <ros/ros.h>
#include <val_task3/val_task3_utils.h>
#include <srcsim/Leak.h>
#include <val_moveit_planners/val_cartesian_planner.h>
#include <val_controllers/val_wholebody_manipulation.h>

#define VERTICAL_WIDTH    0.25
#define HORIZONTAL_WIDTH  0.25

class leakDetector{
private:
    ros::NodeHandle nh_;
    ros::Subscriber leak_sb_;
    ros::Publisher  marker_pub_;
    cartesianPlanner* left_arm_planner_;
    wholebodyManipulation* wholebody_controller_;

    double leak_value_;

    void visulatiseSearchPoints(std::vector<geometry_msgs::Pose> &poses, geometry_msgs::Point horz_left_top, geometry_msgs::Point horz_right_bottom);

public:
    leakDetector(ros::NodeHandle nh);
    ~leakDetector();


    void generateSearchWayPoints(geometry_msgs::Point horz_left_top, geometry_msgs::Point horz_right_bottom, float ver_low_limit, float ver_high_limit, std::vector<geometry_msgs::Pose> &way_points);
    void leakMsgCB(const srcsim::Leak &leakmsg);
    double getLeakValue() const;
    void setLeakValue(double getLeakValue);
    void findLeak (std::vector<geometry_msgs::Pose> &way_points, geometry_msgs::Point& leak_point);
};
