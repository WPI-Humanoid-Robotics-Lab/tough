#ifndef DOOR_OPENER_H
#define DOOR_OPENER_H


#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_controllers/val_wholebody_manipulation.h"
#include "val_controllers/val_gripper_control.h"
#include <val_task_common/val_task_common_utils.h>
#include <tf/transform_datatypes.h>
#include "val_footstep/ValkyrieWalker.h"
#include "val_task3_utils.h"

class doorOpener{

public:
    doorOpener(ros::NodeHandle nh);
    ~doorOpener();

    void openDoor(geometry_msgs::Pose &valveCenterWorld);

private:
    ros::NodeHandle nh_;
    armTrajectory armTraj_;
    gripperControl gripper_;
    task3Utils task3_;
    ValkyrieWalker walker_;
    RobotStateInformer *robot_state_;
};














#endif // DOOR_OPENER_H
