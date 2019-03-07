#ifndef WHOLEBODY_CONTROL_INTERFACE_H
#define WHOLEBODY_CONTROL_INTERFACE_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <ihmc_msgs/WholeBodyTrajectoryRosMessage.h>
#include <ihmc_msgs/SO3TrajectoryPointRosMessage.h>

#include "tough_common/robot_description.h"
#include "tough_common/robot_state.h"
#include "tough_controller_interface/arm_control_interface.h"
#include "tough_controller_interface/chest_control_interface.h"
#include "tough_controller_interface/tough_control_interface.h"

class WholebodyControlInterface : public ToughControlInterface
{
public:
  explicit WholebodyControlInterface(ros::NodeHandle& nh);
  void executeTrajectory(const trajectory_msgs::JointTrajectory& traj);
  void executeTrajectory(const moveit_msgs::RobotTrajectory& traj);

  bool getJointSpaceState(std::vector<double>& joints, RobotSide side) override;

  bool getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side,
                         std::string fixedFrame = TOUGH_COMMON_NAMES::WORLD_TF) override;

private:
  ros::Publisher m_wholebodyPub;

  ChestControlInterface chestController_;
  ArmControlInterface armController_;

  std::vector<std::string> left_arm_joint_names_;
  std::vector<std::string> right_arm_joint_names_;
  std::vector<std::string> chest_joint_names_;

  std::vector<std::pair<double, double>> left_arm_joint_limits_;
  std::vector<std::pair<double, double>> right_arm_joint_limits_;

  void initializeWholebodyMessage(ihmc_msgs::WholeBodyTrajectoryRosMessage& wholeBodyMsg);
  void parseTrajectory(const trajectory_msgs::JointTrajectory& traj,
                       ihmc_msgs::WholeBodyTrajectoryRosMessage& wholeBodyMsg);
  bool validateJointSequenceInTrajectory(const std::vector<std::string>& traj_joint_names,
                                         const std::vector<std::string>& joint_names, long start);

  inline void createChestQuaternion(const long start, const trajectory_msgs::JointTrajectoryPoint& traj_point,
                                    geometry_msgs::Quaternion& quat_msg)
  {
    // chest can only have 3 joints
    double back_bkz = traj_point.positions[start];
    double back_bky = traj_point.positions[start + 1];
    double back_bkx = traj_point.positions[start + 2];
    tf::Quaternion quat;
    quat.setRPY(back_bkx, back_bky, back_bkz);
    tf::quaternionTFToMsg(quat, quat_msg);
  }
};

#endif  // WHOLEBODY_CONTROL_INTERFACE_H
