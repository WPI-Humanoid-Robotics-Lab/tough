#ifndef ROBOT_DESCRIPTION_H
#define ROBOT_DESCRIPTION_H

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <urdf/model.h>
#include <urdf_model/types.h>
#include "tough_common/tough_common_names.h"
#include <geometry_msgs/TransformStamped.h>

/**
 * @brief enum to specify side of the robot. Used for limbs and end-effectors.
 */
enum RobotSide
{
  LEFT = 0,
  RIGHT
};

/**
 * @brief enum for specifying direction.
 */
enum class direction
{
  LEFT = 0,  // Positive Y 0
  RIGHT,     // Negative Y 1
  UP,        // Positive Z 2
  DOWN,      // Negative Z 3
  FRONT,     // Positive X 4
  BACK       // Negative X 5
};

enum class EE_LOADING
{
  LOAD = 0,
  UNLOAD
};

class RobotDescription
{
public:
  /**
   * @brief Get the Robot Description object. RobotDescription is a singleton class and only one object is shared with
   * all the classes that need it. Hence, the consctructor is disabled and this function is provided for accessing
   * pointer to the object.
   *
   * @param nh  Nodehandle
   * @param urdf_param  This argument is only required if urdf is loaded on a different parameter on param server.
   * robot_name variable is prefixed to this parameter.
   * @return RobotDescription*  Pointer to the existing RobotDescription object
   */
  static RobotDescription* getRobotDescription(ros::NodeHandle nh, std::string urdf_param = "/robot_description");

  // disable assign and copy
  RobotDescription(RobotDescription const&) = delete;
  void operator=(RobotDescription const&) = delete;

  /**
   * @brief Get the Pelvis Frame name
   *
   * @return std::string
   */
  std::string getPelvisFrame() const;

  /**
   * @brief Get the World Frame name
   *
   * @return std::string
   */
  std::string getWorldFrame() const;

  /**
   * @brief Get the Torso Frame name
   *
   * @return std::string
   */
  std::string getTorsoFrame() const;

  /**
   * @brief Get the Left Foot Frame Name
   *
   * @return std::string
   */
  std::string getLeftFootFrameName() const;

  /**
   * @brief Get the Right Foot Frame Name
   *
   * @return std::string
   */
  std::string getRightFootFrameName() const;

  /**
   * @brief Get the Left Palm Frame
   *
   * @return std::string
   */
  std::string getLeftPalmFrame() const;

  /**
   * @brief Get the Right Palm Frame name
   *
   * @return std::string
   */
  std::string getRightPalmFrame() const;

  /**
   * @brief Get the Right End Effector Frame name
   *
   * @return std::string
   */
  std::string getRightEEFrame() const;

  /**
   * @brief Get the Left End Effector Frame name
   *
   * @return std::string
   */
  std::string getLeftEEFrame() const;

  /**
   * @brief Get the Robot Name
   *
   * @return std::string
   */
  std::string getRobotName() const;

  /**
   * @brief URDF Parameter name
   *
   * @return std::string
   */
  std::string getURDFParameter() const;

  /**
   * @brief Get the vector of Left Arm Joint Names
   *
   * @param left_arm_joint_names  a vector of std::string [output]
   */
  void getLeftArmJointNames(std::vector<std::string>& left_arm_joint_names) const;

  /**
   * @brief Get the vector of Right Arm Joint Names
   *
   * @param right_arm_joint_names  a vector of std::string [output]
   */
  void getRightArmJointNames(std::vector<std::string>& right_arm_joint_names) const;

  /**
   * @brief Get the vector of Chest Joint Names
   *
   * @param chest_joint_names  a vector of std::string [output]
   */
  void getChestJointNames(std::vector<std::string>& chest_joint_names) const;

  /**
   * @brief Get the vector of Left Arm Frame Names
   *
   * @param left_arm_frame_names  a vector of std::string [output]
   */
  void getLeftArmFrameNames(std::vector<std::string>& left_arm_frame_names) const;

  /**
   * @brief Get the vector of Right Arm Frame Names
   *
   * @param right_arm_frame_names  a vector of std::string [output]
   */
  void getRightArmFrameNames(std::vector<std::string>& right_arm_frame_names) const;

  /**
   * @brief Get the Left Arm Joint Limits
   *
   * @param left_arm_joint_limits  a vector of pair of double for minimum and maximum joint limit [output]
   */
  void getLeftArmJointLimits(std::vector<std::pair<double, double> >& left_arm_joint_limits) const;

  /**
   * @brief Get the Right Arm Joint Limits
   *
   * @param right_arm_joint_limits  a vector of pair of double for minimum and maximum joint limit [output]
   */
  void getRightArmJointLimits(std::vector<std::pair<double, double> >& right_arm_joint_limits) const;

  /**
   * @brief Get the Chest Joint Limits
   *
   * @param chest_joint_limits  a vector of pair of double for minimum and maximum joint limit [output]
   */
  void getChestJointLimits(std::vector<std::pair<double, double> >& chest_joint_limits) const;

  /**
   * @brief Get the Chest Frame Names
   *
   * @param chest_frame_names         - [output] returns the chest frame names
   */
  void getChestFrameNames(std::vector<std::string>& chest_frame_names) const;

  /**
   * @brief Returns the joint names for the joints considered in the Move Groups.
   *
   * @param move_group                - The move group name for which the joint names are needed
   * @return std::vector<std::string> - [output] returns the vector of joint names for the move group.
   */
  const std::vector<std::string> getFrameNamesInMoveGroup(const std::string& move_group);

  /**
   * @brief Get the Parent Frame For Move Groups
   *
   * @param move_group                - The move group name for which the parent frame name is needed
   * @return std::string        - [output] returns the parent frame name for the move_group
   */
  std::string getParentFrameForMoveGroups(const std::string& move_group);

  /**
   * @brief Get the Parent Frame name for the given joint namej
   * 
   * @param joint_name                - Joint Name 
   * @return const std::string        - Returns the parent frame name for the joint name
   */
  std::string getParentFrameForJointName(const std::string& joint_name);

  int getNumberOfNeckJoints() const;

  /**
   * @brief Frame Hash values are used while sending trajectories in a specific frame. This method returns the hash for
   * the Feet frame where the Z axis is always pointing up. (While walking, the local frame might rotate. The
   * 'MidFeetZUP' frame will always point upwards)
   *
   * @return int
   */
  int getMidFeetZUPFrameHash() const;

  /**
   * @brief Frame Hash values are used while sending trajectories in a specific frame. This method returns the hash for
   * the Pelvis frame where the Z axis is always pointing up. (While walking, the local Pelvis frame might rotate/move.
   * The 'PelvisZUP' frame will always point upwards, but still locating at the pelvis frame)
   *
   * @return int
   */
  int getPelvisZUPFrameHash() const;

  /**
   * @brief Frame Hash values are used while sending trajectories in a specific frame. This method returns the hash for
   * the Pelvis Frame (Z axis pointing down).
   *
   * @return int
   */
  int getPelvisFrameHash() const;

  /**
   * @brief Frame Hash values are used while sending trajectories in a specific frame. This method returns the hash for
   * the Chest Frame
   *
   * @return int
   */
  int getChestFrameHash() const;

  /**
   * @brief Frame Hash values are used while sending trajectories in a specific frame. This method returns the hash for
   * the Frame at the Center of Mass.
   *
   * @return int
   */
  int getCenterOfMassFrameHash() const;

  /**
   * @brief Frame Hash values are used while sending trajectories in a specific frame. This method returns the hash for
   * the Frame at the Left Sole of the robot feet.
   *
   * @return int
   */
  int getLeftSoleFrameHash() const;

  /**
   * @brief Frame Hash values are used while sending trajectories in a specific frame. This method returns the hash for
   * the Frame at the Right Sole of the robot feet.
   *
   * @return int
   */
  int getRightSoleFrameHash() const;

  /**
   * @brief Frame Hash values are used while sending trajectories in a specific frame. This method returns the hash for
   * the World Frame.
   *
   * @return int
   */
  int getWorldFrameHash() const;

  /**
   * @brief Foot frame is slightly higher than the world frame when the robot spawns. This offset is required if user
   * wants to get a point on foot ground where the foot touches. Get the Foot Frame Offset object
   *
   * @return double
   */
  double getFootFrameOffset() const;

  /**
   * @brief Get the Foot Frame of required side
   *
   * @param side side of the robot, LEFT or RIGHT
   * @return std::string frame name for the foot of requested side.
   */
  inline std::string getFootFrame(RobotSide side) const
  {
    return side == RobotSide::LEFT ? getLeftFootFrameName() : getRightFootFrameName();
  }

protected:
  void setPelvisFrame(const std::string& value);

  void setTorsoFrame(const std::string& value);

  void setLeftArmJointNames(const std::vector<std::string>& left_arm_joint_names);

  void setRightArmJointNames(const std::vector<std::string>& right_arm_joint_names);

  void setChestJointNames(const std::vector<std::string>& chest_joint_names);

  void setLeftArmFrameNames(const std::vector<std::string>& left_arm_frame_names);

  void setRightArmFrameNames(const std::vector<std::string>& right_arm_frame_names);

  void setChestFrameNames(const std::vector<std::string>& chest_frame_names);

  void setLeftFootFrameName(const std::string& left_foot_frame_name);

  void setRightFootFrameName(const std::string& getRightFootFrameName);

  void setLeftArmJointLimits(const std::vector<std::pair<double, double> >& left_arm_joint_limits);

  void setRightArmJointLimits(const std::vector<std::pair<double, double> >& right_arm_joint_limits);

  void setChestJointLimits(const std::vector<std::pair<double, double> >& chest_joint_limits);

  void setLeftPalmFrame(const std::string& value);

  void setRightPalmFrame(const std::string& value);

  void setNumberOfNeckJoints(int numberOfNeckJoints);

  bool updateFrameHash();

  bool l_palm_exists_ = false;

  bool l_middle_finger_pitch_link_exists_ = false;

  bool l_hand_exists_ = false;

  bool r_palm_exists_ = false;

  bool r_middle_finger_pitch_link_exists_ = false;

  bool r_hand_exists_ = false;

private:
  RobotDescription(ros::NodeHandle nh, std::string urdf_param = "/robot_description");
  ~RobotDescription();
  static RobotDescription* object;
  urdf::Model model_;
  std::vector<urdf::JointSharedPtr> joints_;
  std::vector<urdf::LinkSharedPtr> links_;
  std::string urdf_param_;
  std::string robot_name_;
  std::string param_left_arm_joint_names_;
  std::string param_right_arm_joint_names_;
  std::string param_chest_joint_names_;
  std::string param_left_foot_frame_name_;
  std::string param_right_foot_frame_name_;
  std::string param_left_ee_frame_name_;
  std::string param_right_ee_frame_name_;

  std::string PELVIS_TF;
  std::string TORSO_TF;

  std::string R_PALM_TF;
  std::string L_PALM_TF;

  std::string R_END_EFFECTOR_TF;
  std::string L_END_EFFECTOR_TF;

  std::vector<std::string> left_arm_joint_names_;
  std::vector<std::string> right_arm_joint_names_;
  std::vector<std::string> chest_joint_names_;

  std::vector<std::string> left_arm_frame_names_;
  std::vector<std::string> right_arm_frame_names_;
  std::vector<std::string> chest_frame_names_;

  std::string left_foot_frame_name_;
  std::string right_foot_frame_name_;

  std::vector<std::pair<double, double> > left_arm_joint_limits_;
  std::vector<std::pair<double, double> > right_arm_joint_limits_;
  std::vector<std::pair<double, double> > chest_joint_limits_;

  int number_of_neck_joints_;

  double foot_frame_offset_;

  /* Frame hash - these are defined in us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds
   * Currently there is no way of querying hashID of a frame. Once it is available, it will be implemented in teh
   * constructor of this class
   *    MIDFEET_ZUP_FRAME(-100), PELVIS_ZUP_FRAME(-101),
   *    PELVIS_FRAME(-102), CHEST_FRAME(-103),
   *    CENTER_OF_MASS_FRAME(-104), LEFT_SOLE_FRAME(-105),
   *    RIGHT_SOLE_FRAME(-106);
   */

  int MIDFEET_ZUP_FRAME_HASH_;

  int PELVIS_ZUP_FRAME_HASH_;

  int PELVIS_FRAME_HASH_;

  int CHEST_FRAME_HASH_;

  int CENTER_OF_MASS_FRAME_HASH_;

  int LEFT_SOLE_FRAME_HASH_;

  int RIGHT_SOLE_FRAME_HASH_;

  int WORLD_FRAME_HASH_;

  std::map<std::string, int> frame_hash_map_;
};

#endif  // ROBOT_DESCRIPTION_H
