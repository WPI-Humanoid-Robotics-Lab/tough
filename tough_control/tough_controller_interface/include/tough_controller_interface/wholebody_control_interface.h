#ifndef WHOLEBODYMANIPULATION_H
#define WHOLEBODYMANIPULATION_H

#include <ros/ros.h>
#include "tough_common/robot_description.h"
#include "tough_common/robot_state.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <ihmc_msgs/WholeBodyTrajectoryRosMessage.h>
#include <ihmc_msgs/SO3TrajectoryPointRosMessage.h>
#include "moveit_msgs/RobotTrajectory.h"
#include "tough_controller_interface/tough_controller_interface.h"

class WholebodyControlInterface : public ToughControllerInterface
{
public:
    enum class TrajectoryType {
        INVALID,
        SEVEN_DOF,
        TEN_DOF
    };

    WholebodyControlInterface(ros::NodeHandle &nh);
    void executeTrajectory(const RobotSide side, const trajectory_msgs::JointTrajectory &traj);
    void executeTrajectory(const RobotSide side, const  moveit_msgs::RobotTrajectory &traj);

    virtual bool getJointSpaceState(std::vector<double> &joints, RobotSide side) override;

    virtual bool getTaskSpaceState(geometry_msgs::Pose &pose, RobotSide side, std::string fixedFrame=TOUGH_COMMON_NAMES::WORLD_TF) override;
private :
    ros::Publisher m_wholebodyPub;
    void rightArmMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj,std::vector<std::pair<double, double> > joint_limits_);
    void leftArmMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj, std::vector<std::pair<double, double> > joint_limits_);
    void chestMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj);
    std::vector<std::pair<double, double> > joint_limits_left_;
    std::vector<std::pair<double, double> > joint_limits_right_;
    bool validateTrajectory(const trajectory_msgs::JointTrajectory &traj);
    TrajectoryType getTrajectoryType(const trajectory_msgs::JointTrajectory &traj);
    void jointTrjectoryToArmMessage(const trajectory_msgs::JointTrajectory &traj, ihmc_msgs::ArmTrajectoryRosMessage &msg);
    void generateArmMessage(RobotSide side, const trajectory_msgs::JointTrajectory traj, const   std::vector<std::string> &left_arm_joint_names, ihmc_msgs::ArmTrajectoryRosMessage & msg);
};

#endif // WHOLEBODYMANIPULATION_H
