#ifndef VAL_STAIRCLIMB_H
#define VAL_STAIRCLIMB_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <val_footstep/RobotWalker.h>
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "val_controllers/robot_state.h"
#include "val_controllers/val_chest_navigation.h"
#include <val_task3/stair_detector_2.h>

class StairClimb
{
public:
    StairClimb(ros::NodeHandle n);
    bool walkToSetPosition(geometry_msgs::Pose2D goal);
    RobotWalker* walker_;
    RobotStateInformer* robot_state_;
    chestTrajectory* chest_controller_;
};

#endif // VAL_STAIRCLIMB_H
