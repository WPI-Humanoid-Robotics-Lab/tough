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
    ros::Publisher  leak_loc_pub_;
    ros::Publisher  marker_pub_;
    RobotStateInformer *current_state_;
    task3Utils task3Utils_;
    pelvisTrajectory* pelvis_controller_;
    armTrajectory*  arm_controller_;

    armSide side_;
    bool thumbwards_;

    bool leak_found_;

    void visulatiseSearchPoints(std::vector<geometry_msgs::Pose> &poses, geometry_msgs::Point horz_left_top, geometry_msgs::Point horz_right_bottom);

public:
    leakDetector(ros::NodeHandle nh, armSide side, bool thumbwards);
    ~leakDetector();


    void leakMsgCB(const srcsim::Leak &leakmsg);
    void findLeak();
};
