#ifndef WHOLEBODYMANIPULATION_H
#define WHOLEBODYMANIPULATION_H

#include <ros/ros.h>
#include "tough_common/val_common_defines.h"
#include "tough_common/robot_description.h"
#include "tough_controller_interface/robot_state.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <ihmc_msgs/WholeBodyTrajectoryRosMessage.h>
#include <ihmc_msgs/SO3TrajectoryPointRosMessage.h>
#include "moveit_msgs/RobotTrajectory.h"

class wholebodyManipulation
{
public:
    wholebodyManipulation(ros::NodeHandle &nh);
    void compileMsg(const armSide side, const trajectory_msgs::JointTrajectory &traj);
    void compileMsg(const armSide side, const  moveit_msgs::RobotTrajectory &traj);
private :
    ros::NodeHandle nh_;
    ros::Publisher m_wholebodyPub;
    RobotStateInformer* robot_state_;
    RobotDescription* rd_;
    void rightArmMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj,std::vector<std::pair<float, float> > joint_limits_);
    void leftArmMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj,std::vector<std::pair<float, float> > joint_limits_);
    void chestMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj);
    std::vector<std::pair<float, float> > joint_limits_left_;
    std::vector<std::pair<float, float> > joint_limits_right_;
    bool validateTrajectory(const trajectory_msgs::JointTrajectory &traj);
};

#endif // WHOLEBODYMANIPULATION_H
