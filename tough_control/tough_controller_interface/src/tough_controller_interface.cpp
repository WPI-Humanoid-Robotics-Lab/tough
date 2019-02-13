#include "tough_controller_interface/tough_controller_interface.h"

long ToughControllerInterface::id_ = 1;

ToughControllerInterface::ToughControllerInterface(ros::NodeHandle nh)
{
  if (!nh.getParam(TOUGH_COMMON_NAMES::ROBOT_NAME_PARAM, robot_name_))
  {
    ROS_ERROR("%s parameter is not on the server. Using valkyrie by default", TOUGH_COMMON_NAMES::ROBOT_NAME_PARAM.c_str());
    robot_name_ = "valkyrie";
  }

  control_topic_prefix_ = TOUGH_COMMON_NAMES::TOPIC_PREFIX + robot_name_ + TOUGH_COMMON_NAMES::CONTROL_TOPIC_PREFIX;
  output_topic_prefix_ = TOUGH_COMMON_NAMES::TOPIC_PREFIX + robot_name_ + TOUGH_COMMON_NAMES::OUTPUT_TOPIC_PREFIX;

  state_informer_ = RobotStateInformer::getRobotStateInformer(nh_);
  rd_ = RobotDescription::getRobotDescription(nh_);
}

ToughControllerInterface::~ToughControllerInterface()
{
}
