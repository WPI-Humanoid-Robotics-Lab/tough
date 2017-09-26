#ifndef VAL_STAIRCLIMB_H
#define VAL_STAIRCLIMB_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <val_footstep/ValkyrieWalker.h>
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "tough_controller_interface/robot_state.h"
#include "tough_controller_interface/chest_control_interface.h"
#include <val_task3/stair_detector_2.h>

class StairClimb
{
public:
    StairClimb(ros::NodeHandle n);
    bool walkToSetPosition(geometry_msgs::Pose2D goal);
    ValkyrieWalker* walker_;
    RobotStateInformer* robot_state_;
    chestTrajectory* chest_controller_;
};

#endif // VAL_STAIRCLIMB_H
