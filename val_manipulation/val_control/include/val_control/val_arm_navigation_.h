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
    ros::Subscriber armTrajectorySunscriber;
    ihmc_msgs::ArmTrajectoryRosMessage appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage msg, float time, float* pos);

    static int arm_id;
    const float ZERO_POSE [] ={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const float WALK_POSE [] ={-0.2, 1.2, 0.7222, 1.5101, 0.0, 0.0, 0.0};


public:
    armTrajectory(ros::NodeHandle nh);
    ~armTrajectory();

    void buttonPressArm(armSide side);
    void walkPoseArm(armSide side);
    void zeroPoseArm(armSide side);
    void moveArm(armSide side,std::vector<float> arm_pose, float time);
    void moveArmMessage(armSide side, ihmc_msgs::ArmTrajectoryRosMessage msg);
   // void getArmTrajectorty(armSide side, ihmc_msgs::ArmTrajectoryRosMessage arm);
};
    int armTrajectory::arm_id = -1;
