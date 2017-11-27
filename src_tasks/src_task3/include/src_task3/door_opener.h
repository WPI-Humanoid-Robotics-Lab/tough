#ifndef DOOR_OPENER_H
#define DOOR_OPENER_H


#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "tough_moveit_planners/tough_cartesian_planner.h"
#include "tough_controller_interface/wholebody_control_interface.h"
#include "tough_controller_interface/gripper_control_interface.h"
#include <src_task_common/val_task_common_utils.h>
#include <tf/transform_datatypes.h>
#include "tough_footstep/RobotWalker.h"
#include "val_task3_utils.h"
#include "tough_control_common/tough_control_common.h"

class DoorOpener{

public:
    DoorOpener(ros::NodeHandle nh);
    ~DoorOpener();

    void openDoor(geometry_msgs::Pose &valveCenterWorld);

private:
    ros::NodeHandle nh_;
    ArmControlInterface armTraj_;
    gripperControl gripper_;
    task3Utils task3_;
    RobotWalker walker_;
    RobotStateInformer *robot_state_;
    RobotDescription *rd_;
    valControlCommon control_common_;
};














#endif // DOOR_OPENER_H
