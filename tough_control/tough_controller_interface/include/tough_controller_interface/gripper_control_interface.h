#ifndef TOUGH_GRIPPER_CONTROL_H
#define TOUGH_GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "ihmc_msgs/HandDesiredConfigurationRosMessage.h"
#include <tough_common/robot_description.h>
#include "tough_controller_interface/tough_control_interface.h"
#include <map>

// Note: HOOK Mode doesn't work. Tested on actual robotiq gripper

/**
 * @brief gripperControl class provides ability to control the grippers.
 *
 */
class GripperControlInterface : public ToughControlInterface
{
private:
  ros::Publisher gripperPublisher_;

public:
  /**
   * @brief gripperControl class provides ability to control the grippers.
   * @param nh nodehandle to which subscribers and publishers are attached.
   */
  GripperControlInterface(ros::NodeHandle nh);
  ~GripperControlInterface();

/**
 * @brief enum for the grippers,  (GRIPPER FUNCTIONALITY EXPLAINED ACCORDING TO 
 * THE ROBOTIQ GRIPPERS)
 * BASIC            Simple grip, where the thumb and fingers are at natural positions
 * PINCH            Fingers lie touching each other, where the grasping pose consists of 
 *                  grip between thumb and fingers
 * WIDE             Fingers wide spread, and thumb at natural position
 * SCISSOR          Unmoved thumb, and closing gripper will only close the fingers
 * HOOK             THIS DOES NOT WORK !!
 * RESET            This will reset the gripper mode, and calibrate it
 * 
 */
  enum GRIPPER_MODES
  {
    BASIC = ihmc_msgs::HandDesiredConfigurationRosMessage::BASIC_GRIP,
    PINCH = ihmc_msgs::HandDesiredConfigurationRosMessage::PINCH_GRIP,
    WIDE = ihmc_msgs::HandDesiredConfigurationRosMessage::WIDE_GRIP,
    SCISSOR = ihmc_msgs::HandDesiredConfigurationRosMessage::SCISSOR_GRIP,
    HOOK = ihmc_msgs::HandDesiredConfigurationRosMessage::HOOK,
    RESET = ihmc_msgs::HandDesiredConfigurationRosMessage::RESET
  };

  const std::map<GRIPPER_MODES, std::string> GRIPPER_MODE_NAMES = {
    { GRIPPER_MODES::BASIC, "BASIC" },     { GRIPPER_MODES::PINCH, "PINCH" }, { GRIPPER_MODES::WIDE, "WIDE" },
    { GRIPPER_MODES::SCISSOR, "SCISSOR" }, { GRIPPER_MODES::HOOK, "HOOK" },   { GRIPPER_MODES::RESET, "RESET" }
  };

  /**
   * @brief controlGripper provides the ability to move the grippers to a desired position.
   * @param side is either LEFT or RIGHT
   * @param configuration is an enum value from ihmc_msgs::HandDesiredConfigurationRosMessage.
   */
  void controlGripper(const RobotSide side,
                      int configuration = ihmc_msgs::HandDesiredConfigurationRosMessage::BASIC_GRIP);

  void generateGripperMessage(const RobotSide side, const int configuration,
                              ihmc_msgs::HandDesiredConfigurationRosMessage& msg);
  /**
   * @brief closeGripper closes the gripper completely.
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   */
  void closeGripper(const RobotSide side);

  /**
   * @brief openGripper opens the gripper completely
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   */
  void openGripper(const RobotSide side);

  /**
   * @brief Set the Mode 
   *
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @param mode              Mode for the gripper to be set
   */
  void setMode(const RobotSide side, const GRIPPER_MODES mode);

  /**
   * @brief Resets the gripper mode and calibrates it.
   *
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   */
  void resetGripper(const RobotSide side);

  /**
   * @brief Opens only the thumb of the gripper of the side of the robot
   * 
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   */
  void openThumb(const RobotSide side);

  /**
   * @brief Closes only the thumb of the gripper of the side of the robot
   *
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   */
  void closeThumb(const RobotSide side);

  /**
   * @brief Opens only the fingers of the gripper of the side of the robot
   *
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   */
  void openFingers(const RobotSide side);

  /**
   * @brief Closes only the fingers of the gripper of the side of the robot
   *
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   */
  void closeFingers(const RobotSide side);

  /**
   * @brief Crushes the object with full availbale torque.
   *
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   */
  void crush(const RobotSide side);

  /**
   * @brief Returns the string formatted mode name for the gripper mode
   * 
   * @param mode              mode of the gripper
   * @return std::string      string formatted mode name for the gripper
   */
  std::string getModeName(const GRIPPER_MODES mode) const;

  /**
   * @brief Get the Joint Position for the joint of jointName. The order for
   * the Joints' Names, Numbers, Positions, Velocities and Efforts in their
   * vectors are same.
   *
   * @param joints            [output]
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @return true             When Successful
   * @return false
   */
  virtual bool getJointSpaceState(std::vector<double>& joints, RobotSide side) override;

  /**
   * @brief Get the current pose for the end effector frame with respect of fixedFrame
   *
   * @param pose              [output]
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @param fixedFrame        Reference frame for the pose transformation
   * @return true             When Successful
   * @return false
   */
  virtual bool getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side,
                                 std::string fixedFrame = TOUGH_COMMON_NAMES::WORLD_TF) override;
};

#endif  // TOUGH_GRIPPER_CONTROL_H
