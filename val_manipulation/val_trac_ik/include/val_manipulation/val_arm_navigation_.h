#pragma once

#include <ros/ros.h>
#include <ihmc_msgs/ArmTrajectoryRosMessage.h>
#include <ihmc_msgs/OneDoFJointTrajectoryRosMessage.h>
#include <ihmc_msgs/TrajectoryPoint1DRosMessage.h>
#include <ihmc_msgs/HandDesiredConfigurationRosMessage.h>
#include <val_common/val_common_defines.h>

class armTrajectory {

private:

    ros::NodeHandle nh_;
    ros::Publisher armTrajectoryPublisher;
    ros::Publisher handTrajectoryPublisher;
    ihmc_msgs::ArmTrajectoryRosMessage appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage msg, float time, float* pos);

public:
    armTrajectory(ros::NodeHandle nh);
    ~armTrajectory();

    void buttonPressPrepareArm(armSide side);
    void buttonPressArm(armSide side);
    void walkPoseArm(armSide side);
    void zeroPoseArm(armSide side);
};
