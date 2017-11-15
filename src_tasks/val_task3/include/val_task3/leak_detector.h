# pragma once

#include <ros/ros.h>
#include <val_task3/val_task3_utils.h>
#include <srcsim/Leak.h>
#include <tough_moveit_planners/tough_cartesian_planner.h>
#include <tough_controller_interface/wholebody_control_interface.h>
#include "tough_common/robot_description.h"

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
    RobotDescription* rd_;

    RobotSide side_;
    bool thumbwards_;

    bool leak_found_;

    void visulatiseSearchPoints(std::vector<geometry_msgs::Pose> &poses, geometry_msgs::Point horz_left_top, geometry_msgs::Point horz_right_bottom);

public:
    leakDetector(ros::NodeHandle nh, RobotSide side, bool thumbwards);
    ~leakDetector();


    void leakMsgCB(const srcsim::Leak &leakmsg);
    void findLeak();
};
