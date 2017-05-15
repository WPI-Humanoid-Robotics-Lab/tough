#ifndef WHOLEBODYMANIPULATION_H
#define WHOLEBODYMANIPULATION_H

#include <ros/ros.h>
#include "val_common/val_common_defines.h"
#include "val_control/robot_state.h"
#include <trajectory_msgs/JointTrajectory.h>
#include<val_control/val_arm_navigation.h>
#include <val_control/val_chest_navigation.h>
#include <ihmc_msgs/WholeBodyTrajectoryRosMessage.h>
#include <ihmc_msgs/SO3TrajectoryPointRosMessage.h>


class wholebodyManipulation
{
public:
    wholebodyManipulation(ros::NodeHandle &nh);
    void compileMsg(const armSide side, const trajectory_msgs::JointTrajectory &traj);
private :
    ros::NodeHandle nh_;
    ros::Publisher m_wholebodyPub;
    RobotStateInformer* robot_state_;
    void rightArmMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj,std::vector<std::pair<float, float> > joint_limits_);
    void leftArmMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj,std::vector<std::pair<float, float> > joint_limits_);
    void chestMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj);
    std::vector<std::pair<float, float> > joint_limits_left_;
    std::vector<std::pair<float, float> > joint_limits_right_;
    bool validateTrajectory(const trajectory_msgs::JointTrajectory &traj);
};

#endif // WHOLEBODYMANIPULATION_H
