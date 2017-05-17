#ifndef VAL_STAIRCLIMB_H
#define VAL_STAIRCLIMB_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <val_footstep/ValkyrieWalker.h>
class StairClimb
{
public:
    StairClimb(ros::NodeHandle n);
    bool walkToSetPosition(geometry_msgs::Pose2D goal);
    bool takeStep(armSide side, float stepLength, float stepHeight);
    ValkyrieWalker* walker_;
};

#endif // VAL_STAIRCLIMB_H
