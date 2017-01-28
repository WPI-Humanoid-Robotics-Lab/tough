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
    ihmc_msgs::ArmTrajectoryRosMessage appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage &msg, float time, std::vector<float> pos);

    static int arm_id;
    const std::vector<float> ZERO_POSE;
    const std::vector<float> DEFAULT_POSE;


public:
    armTrajectory(ros::NodeHandle nh);
    ~armTrajectory();

    struct moveArmData {
        armSide side;
        std::vector<float> arm_pose;
        float time;
    };

    //void buttonPressArm(armSide side);
    void walkPoseArm(armSide side);
    void zeroPoseArm(armSide side);
    void moveArm(const armSide side,const std::vector<std::vector<float> > arm_pose,const float time);
    void moveArm(std::vector<moveArmData> arm_data);
    void moveArmMessage(ihmc_msgs::ArmTrajectoryRosMessage& msg);
   // void getArmTrajectorty(armSide side, ihmc_msgs::ArmTrajectoryRosMessage arm);
};


#endif // VAL_ARM_NAVIGATION_H
