#include "tough_common/robot_description.h"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <string>
#include <cctype>

// The following function to find substring is copied from stack overflow
// Try to find in the Haystack the Needle - ignore case
bool findSubStringIC(const std::string& strHaystack, const std::string& strNeedle)
{
  auto it = std::search(strHaystack.begin(), strHaystack.end(), strNeedle.begin(), strNeedle.end(),
                        [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); });
  return (it != strHaystack.end());
}

// define static variables
RobotDescription* RobotDescription::object = nullptr;

RobotDescription* RobotDescription::getRobotDescription(ros::NodeHandle nh, std::string urdf_param)
{
  if (RobotDescription::object == nullptr)
  {
    static RobotDescription obj(nh, urdf_param);
    object = &obj;
  }
  return object;
}

RobotDescription::RobotDescription(ros::NodeHandle nh, std::string urdf_param)
{
  param_left_arm_joint_names_ = TOUGH_COMMON_NAMES::LEFT_ARM_JOINT_NAMES_PARAM;
  param_right_arm_joint_names_ = TOUGH_COMMON_NAMES::RIGHT_ARM_JOINT_NAMES_PARAM;
  param_chest_joint_names_ = TOUGH_COMMON_NAMES::CHEST_JOINT_NAMES_PARAM;
  param_left_foot_frame_name_ = TOUGH_COMMON_NAMES::LEFT_FOOT_FRAME_NAME_PARAM;
  param_right_foot_frame_name_ = TOUGH_COMMON_NAMES::RIGHT_FOOT_FRAME_NAME_PARAM;
  param_left_ee_frame_name_ = TOUGH_COMMON_NAMES::LEFT_EE_FRAME_NAME_PARAM;
  param_right_ee_frame_name_ = TOUGH_COMMON_NAMES::RIGHT_EE_FRAME_NAME_PARAM;

  if (!nh.getParam(TOUGH_COMMON_NAMES::ROBOT_NAME_PARAM, robot_name_))
  {
    ROS_ERROR("Could not read robot_name");
  }

  ROS_INFO("Robot Name : %s", robot_name_.c_str());
  std::string prefix = TOUGH_COMMON_NAMES::TOPIC_PREFIX + robot_name_ + "/";

  std::string robot_xml;
  urdf_param = "/" + robot_name_ + urdf_param;
  if (!nh.getParam(urdf_param, robot_xml) || !model_.initString(robot_xml))
  {
    ROS_ERROR("Could not read the robot_description");
    return;
  }
  urdf_param_ = urdf_param;
  if (robot_name_ == "")
  {
    robot_name_.assign(model_.getName());
  }

  param_left_arm_joint_names_.insert(0, prefix);
  param_right_arm_joint_names_.insert(0, prefix);
  param_chest_joint_names_.insert(0, prefix);
  param_left_foot_frame_name_.insert(0, prefix);
  param_right_foot_frame_name_.insert(0, prefix);
  param_left_ee_frame_name_.insert(0, prefix);
  param_right_ee_frame_name_.insert(0, prefix);

  if (!(nh.getParam(param_left_arm_joint_names_, left_arm_joint_names_) &&
        nh.getParam(param_right_arm_joint_names_, right_arm_joint_names_) &&
        nh.getParam(param_chest_joint_names_, chest_joint_names_) &&
        nh.getParam(param_left_foot_frame_name_, left_foot_frame_name_) &&
        nh.getParam(param_right_foot_frame_name_, right_foot_frame_name_) &&
        nh.getParam(param_right_ee_frame_name_, R_END_EFFECTOR_TF) &&
        nh.getParam(param_left_ee_frame_name_, L_END_EFFECTOR_TF)))
  {
    ROS_ERROR("Could not read the joint names from parameter server. Check if you have the latest ihmc_%s_ros package",
              robot_name_.c_str());
    return;
  }

  // get a vector of all links
  model_.getLinks(links_);

  // set joint limits for arms
  for (auto joint_name : left_arm_joint_names_)
  {
    float l_limit = model_.joints_[joint_name]->limits->lower;
    float u_limit = model_.joints_[joint_name]->limits->upper;
    left_arm_joint_limits_.push_back({ l_limit, u_limit });
    left_arm_frame_names_.push_back(model_.joints_[joint_name]->child_link_name);
  }

  for (auto joint_name : right_arm_joint_names_)
  {
    float l_limit = model_.joints_[joint_name]->limits->lower;
    float u_limit = model_.joints_[joint_name]->limits->upper;
    right_arm_joint_limits_.push_back({ l_limit, u_limit });
    right_arm_frame_names_.push_back(model_.joints_[joint_name]->child_link_name);
  }

  for (auto joint_name : chest_joint_names_)
  {
    float l_limit = model_.joints_[joint_name]->limits->lower;
    float u_limit = model_.joints_[joint_name]->limits->upper;
    chest_joint_limits_.push_back({ l_limit, u_limit });
    chest_frame_names_.push_back(model_.joints_[joint_name]->child_link_name);
  }

  for (auto link : links_)
  {
    // assuming that pelvis frame will be the one with least length
    if (findSubStringIC(link->name, "pelvis") && (link->name.length() < PELVIS_TF.length() || PELVIS_TF.empty()))
    {
      PELVIS_TF = link->name;
    }
  }

  L_PALM_TF = *(left_arm_frame_names_.end() - 1);
  R_PALM_TF = *(right_arm_frame_names_.end() - 1);
  TORSO_TF = model_.joints_[left_arm_joint_names_[0]]->parent_link_name;

  /* With 0.11 version of open-robotics-software, the foot offset is not handled on JAVA side
   * This causes footsteps to be at a height from ground. should this be fixed on JAVA side?
   */

  if (robot_name_ == "atlas")
  {
    number_of_neck_joints_ = 1;
    foot_frame_offset_ = 0.085;
  }
  else if (robot_name_ == "valkyrie")
  {
    number_of_neck_joints_ = 3;
    foot_frame_offset_ = 0.102;
  }

  updateFrameHash();

  ROS_INFO("Left foot frame : %s", left_foot_frame_name_.c_str());
  ROS_INFO("Right foot frame : %s", right_foot_frame_name_.c_str());
  ROS_INFO("Pelvis Frame : %s", PELVIS_TF.c_str());
  ROS_INFO("Torso Frame : %s", TORSO_TF.c_str());
  ROS_INFO("Right Palm Frame : %s", R_PALM_TF.c_str());
  ROS_INFO("Left Palm Frame : %s", L_PALM_TF.c_str());
  ROS_INFO("Right EE Frame : %s", R_END_EFFECTOR_TF.c_str());
  ROS_INFO("Left EE Frame : %s", L_END_EFFECTOR_TF.c_str());
}

RobotDescription::~RobotDescription()
{
}

double RobotDescription::getFootFrameOffset() const
{
  return foot_frame_offset_;
}

int RobotDescription::getRightSoleFrameHash() const
{
  return RIGHT_SOLE_FRAME_HASH_;
}

int RobotDescription::getWorldFrameHash() const
{
  return WORLD_FRAME_HASH_;
}

int RobotDescription::getLeftSoleFrameHash() const
{
  return LEFT_SOLE_FRAME_HASH_;
}

int RobotDescription::getCenterOfMassFrameHash() const
{
  return CENTER_OF_MASS_FRAME_HASH_;
}

int RobotDescription::getChestFrameHash() const
{
  return CHEST_FRAME_HASH_;
}

int RobotDescription::getPelvisFrameHash() const
{
  return PELVIS_FRAME_HASH_;
}

int RobotDescription::getPelvisZUPFrameHash() const
{
  return PELVIS_ZUP_FRAME_HASH_;
}

int RobotDescription::getMidFeetZUPFrameHash() const
{
  return MIDFEET_ZUP_FRAME_HASH_;
}

int RobotDescription::getNumberOfNeckJoints() const
{
  return number_of_neck_joints_;
}

void RobotDescription::setNumberOfNeckJoints(int numberOfNeckJoints)
{
  number_of_neck_joints_ = numberOfNeckJoints;
}

bool RobotDescription::updateFrameHash()
{
  MIDFEET_ZUP_FRAME_HASH_ = TOUGH_COMMON_NAMES::MIDFEET_ZUP_FRAME_HASH;
  PELVIS_ZUP_FRAME_HASH_ = TOUGH_COMMON_NAMES::PELVIS_ZUP_FRAME_HASH;
  PELVIS_FRAME_HASH_ = TOUGH_COMMON_NAMES::PELVIS_FRAME_HASH;
  CHEST_FRAME_HASH_ = TOUGH_COMMON_NAMES::CHEST_FRAME_HASH;
  CENTER_OF_MASS_FRAME_HASH_ = TOUGH_COMMON_NAMES::CENTER_OF_MASS_FRAME_HASH;
  LEFT_SOLE_FRAME_HASH_ = TOUGH_COMMON_NAMES::LEFT_SOLE_FRAME_HASH;
  RIGHT_SOLE_FRAME_HASH_ = TOUGH_COMMON_NAMES::RIGHT_SOLE_FRAME_HASH;
  WORLD_FRAME_HASH_ = TOUGH_COMMON_NAMES::WORLD_FRAME_HASH;
}

const std::string RobotDescription::getRightPalmFrame() const
{
  return R_PALM_TF;
}

const std::string RobotDescription::getRightEEFrame() const
{
  return R_END_EFFECTOR_TF;
}

const std::string RobotDescription::getLeftEEFrame() const
{
  return L_END_EFFECTOR_TF;
}

const std::string RobotDescription::getRobotName() const
{
  return robot_name_;
}

const std::string RobotDescription::getURDFParameter() const
{
  return urdf_param_;
}
void RobotDescription::setRightPalmFrame(const std::string& value)
{
  R_PALM_TF = value;
}

const std::string RobotDescription::getLeftPalmFrame() const
{
  return L_PALM_TF;
}

void RobotDescription::setLeftPalmFrame(const std::string& value)
{
  L_PALM_TF = value;
}

void RobotDescription::getRightArmJointLimits(std::vector<std::pair<double, double> >& right_arm_joint_limits) const
{
  right_arm_joint_limits = right_arm_joint_limits_;
}

void RobotDescription::setRightArmJointLimits(const std::vector<std::pair<double, double> >& right_arm_joint_limits)
{
  right_arm_joint_limits_.assign(right_arm_joint_limits.begin(), right_arm_joint_limits.end());
}

void RobotDescription::getLeftArmJointLimits(std::vector<std::pair<double, double> >& left_arm_joint_limits) const
{
  left_arm_joint_limits = left_arm_joint_limits_;
}

void RobotDescription::setLeftArmJointLimits(const std::vector<std::pair<double, double> >& left_arm_joint_limits)
{
  left_arm_joint_limits_.assign(left_arm_joint_limits.begin(), left_arm_joint_limits.end());
}

void RobotDescription::getChestJointLimits(std::vector<std::pair<double, double> >& chest_joint_limits) const
{
  chest_joint_limits = chest_joint_limits_;
}

void RobotDescription::setChestJointLimits(const std::vector<std::pair<double, double> >& chest_joint_limits)
{
  chest_joint_limits_.assign(chest_joint_limits.begin(), chest_joint_limits.end());
}

const std::string RobotDescription::getRightFootFrameName() const
{
  return right_foot_frame_name_;
}

void RobotDescription::setRightFootFrameName(const std::string& right_foot_frame_name)
{
  right_foot_frame_name_ = right_foot_frame_name;
}

const std::string RobotDescription::getLeftFootFrameName() const
{
  return left_foot_frame_name_;
}

void RobotDescription::setLeftFootFrameName(const std::string& left_foot_frame_name)
{
  left_foot_frame_name_ = left_foot_frame_name;
}

void RobotDescription::getRightArmFrameNames(std::vector<std::string>& right_arm_frame_names) const
{
  right_arm_frame_names = right_arm_frame_names_;
}

void RobotDescription::setRightArmFrameNames(const std::vector<std::string>& right_arm_frame_names)
{
  right_arm_frame_names_.assign(right_arm_frame_names.begin(), right_arm_frame_names.end());
}

void RobotDescription::getLeftArmFrameNames(std::vector<std::string>& left_arm_frame_names) const
{
  left_arm_frame_names = left_arm_frame_names_;
}

void RobotDescription::setLeftArmFrameNames(const std::vector<std::string>& left_arm_frame_names)
{
  left_arm_frame_names_.assign(left_arm_frame_names.begin(), left_arm_frame_names.end());
}

void RobotDescription::getChestFrameNames(std::vector<std::string>& chest_frame_names) const
{
  chest_frame_names = chest_frame_names_;
}

void RobotDescription::setChestFrameNames(const std::vector<std::string>& chest_frame_names)
{
  chest_frame_names_.assign(chest_frame_names.begin(), chest_frame_names.end());
}

void RobotDescription::getRightArmJointNames(std::vector<std::string>& right_arm_joint_names) const
{
  right_arm_joint_names = right_arm_joint_names_;
}

void RobotDescription::getChestJointNames(std::vector<std::string>& chest_joint_names) const
{
  chest_joint_names = chest_joint_names_;
}

void RobotDescription::setRightArmJointNames(const std::vector<std::string>& right_arm_joint_names)
{
  right_arm_joint_names_.assign(right_arm_joint_names.begin(), right_arm_joint_names.end());
}

void RobotDescription::setChestJointNames(const std::vector<std::string>& chest_joint_names)
{
  chest_joint_names_.assign(chest_joint_names.begin(), chest_joint_names.end());
}

void RobotDescription::getLeftArmJointNames(std::vector<std::string>& left_arm_joint_names) const
{
  left_arm_joint_names = left_arm_joint_names_;
}

void RobotDescription::setLeftArmJointNames(const std::vector<std::string>& left_arm_joint_names)
{
  left_arm_joint_names_.assign(left_arm_joint_names.begin(), left_arm_joint_names.end());
}

const std::string RobotDescription::getTorsoFrame() const
{
  return TORSO_TF;
}

void RobotDescription::setTorsoFrame(const std::string& value)
{
  TORSO_TF = value;
}

const std::string RobotDescription::getPelvisFrame() const
{
  return PELVIS_TF;
}

const std::string RobotDescription::getWorldFrame() const
{
  return TOUGH_COMMON_NAMES::WORLD_TF;
}

void RobotDescription::setPelvisFrame(const std::string& value)
{
  PELVIS_TF = value;
}

const std::vector<std::string> RobotDescription::getFrameNamesInMoveGroup(const std::string& move_group)
{
  std::vector<std::string> frame_names;
  std::vector<std::string> chest_frame_names, arm_frame_names, arm_joint_names;
  std::string end_effector_frame;
  chest_frame_names.resize(0);

  if(move_group == TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP)
  {
    this->getLeftArmFrameNames(arm_frame_names);
    end_effector_frame = this->getLeftEEFrame();
  }
  else if(move_group == TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP)
  {
    this->getRightArmFrameNames(arm_frame_names);
    end_effector_frame = this->getRightEEFrame();
  }
  else if(move_group == TOUGH_COMMON_NAMES::LEFT_ARM_10DOF_GROUP)
  {
    this->getLeftArmFrameNames(arm_frame_names);
    this->getChestFrameNames(chest_frame_names);
    end_effector_frame = this->getLeftEEFrame();
  }
  else if(move_group == TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP)
  {
    this->getRightArmFrameNames(arm_frame_names);
    this->getChestFrameNames(chest_frame_names);
    end_effector_frame = this->getRightEEFrame();
  }

  frame_names = chest_frame_names;
  frame_names.insert(frame_names.end(), arm_frame_names.begin(), arm_frame_names.end());

  // check valkyrie frames if it has an additional frame between end effector and the end of arm
  
  //An intermediate palm frame exists between end_effector_frame and the last frame in the arm joints. 
  std::string last_frame_name = frame_names.back();
  std::string intermediate_frame_name = model_.links_[last_frame_name]->child_joints.at(0)->child_link_name;
  
  //add intermediate frame only if the parent of end_effector frame is same as the intermediate frame.
  if (intermediate_frame_name == model_.links_[end_effector_frame]->getParent()->name)
  {
    frame_names.push_back(intermediate_frame_name);
  }

  //To add the end effector frame to the frame_names vector, if not already there
  if (end_effector_frame != frame_names.back())
  {
    frame_names.push_back(end_effector_frame);
  }

  return frame_names;
}

const std::string RobotDescription::getParentFrameForMoveGroups(const std::string& move_group)
{
  std::vector<std::string> frame_names = this->getFrameNamesInMoveGroup(move_group);
  std::string first_frame = frame_names.front();
  std::string parent_frame = model_.links_[first_frame]->getParent()->name;
  return parent_frame;
}

const std::string RobotDescription::getParentFrameForJointName(const std::string& joint_name)
{
  std::string parent_frame = model_.links_[joint_name]->getParent()->name;
  return parent_frame;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_urdf");
  ros::NodeHandle nh;
  RobotDescription* robot = RobotDescription::getRobotDescription(nh);
  
  return 0;
}
