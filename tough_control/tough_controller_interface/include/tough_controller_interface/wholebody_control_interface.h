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

/**
 * @brief  The WholebodyControlInterface class provides ability to control whole body of humanoid robots supported by
 * open-humanoids-software
 *
 */
class WholebodyControlInterface : public ToughControlInterface
{
public:
  /**
   * @brief The WholebodyControlInterface class provides ability to control whole body of humanoid robots supported by
   * open-humanoids-software
   *
   * @param nh    nodehandle to which subscribers and publishers are attached.
   */
  explicit WholebodyControlInterface(ros::NodeHandle& nh);

  /**
   * @brief This method executes the trajectory on the Robot
   *
   * @param traj                      JointTrajectory message to be executed on the robot.
   */
  void executeTrajectory(const trajectory_msgs::JointTrajectory& traj);

  /**
   * @brief This method executes the trajectory on the Robot
   *
   * @param traj                      RobotTrajectory to be executed on the robot
   */
  void executeTrajectory(const moveit_msgs::RobotTrajectory& traj);

  /**
   * @brief Get the current positions of all joints of the side of the chest.
   * Ordering is based on the order in the JointNames vector. The order for the Joints' Names, Numbers,
   * Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param joints                    [output]
   * @return true                     When Successful
   * @return false
   */
  bool getJointSpaceState(std::vector<double>& joints, RobotSide side) override;

  /**
   * @brief Get the current positions of all joints of the side of the chest.
   * Ordering is based on the order in the JointNames vector. The order for the Joints' Names, Numbers,
   * Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param pose              [output]
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @param fixedFrame        Reference frame for the state query
   * @return true             When successful
   * @return false
   */
  bool getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side,
                         std::string fixedFrame = TOUGH_COMMON_NAMES::WORLD_TF) override;

private:
  ros::Publisher m_wholebodyPub;

  ChestControlInterface chestController_;
  ArmControlInterface armController_;

  std::vector<std::string> left_arm_joint_names_;
  std::vector<std::string> right_arm_joint_names_;
  std::vector<std::string> chest_joint_names_;

  std::vector<double> joint_positions_;

  int chest_start_index_, left_arm_start_index_, right_arm_start_index_;

  void initializeWholebodyMessage(ihmc_msgs::WholeBodyTrajectoryRosMessage& wholeBodyMsg);
  void parseTrajectory(const trajectory_msgs::JointTrajectory& traj,
                       ihmc_msgs::WholeBodyTrajectoryRosMessage& wholeBodyMsg);
  
    inline geometry_msgs::Quaternion getChestQuaternion(const long start, const std::vector<double>& traj_vector)
  {
    geometry_msgs::Quaternion quat_msg;
    // chest can only have 3 joints
    double back_bkz = traj_vector.at(start);
    double back_bky = traj_vector.at(start + 1);
    double back_bkx = traj_vector.at(start + 2);
    tf::Quaternion quat;
    quat.setRPY(back_bkx, back_bky, back_bkz);
    tf::quaternionTFToMsg(quat, quat_msg);
    return quat_msg;
  }
};

#endif  // WHOLEBODY_CONTROL_INTERFACE_H
