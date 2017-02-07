#ifndef VAL_ARM_NAVIGATION_H
#define VAL_ARM_NAVIGATION_H

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
    void appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage &msg, float time, std::vector<float> pos);

    static int arm_id;
    const std::vector<float> ZERO_POSE;
    const std::vector<float> DEFAULT_POSE;


public:
    armTrajectory(ros::NodeHandle nh);
    ~armTrajectory();

    struct armJointData {
        armSide side;
        std::vector<float> arm_pose;
        float time;
    };

    //void buttonPressArm(armSide side);
    void moveToDefaultPose(armSide side);
    void moveToZeroPose(armSide side);
    void moveArmJoints(const armSide side,const std::vector<std::vector<float> > arm_pose,const float time);
    void moveArmJoints(std::vector<armJointData> arm_data);
    void moveArmMessage(ihmc_msgs::ArmTrajectoryRosMessage& msg);
   // void getArmTrajectorty(armSide side, ihmc_msgs::ArmTrajectoryRosMessage arm);
};

int armTrajectory::arm_id = -1;
#endif // VAL_ARM_NAVIGATION_H
