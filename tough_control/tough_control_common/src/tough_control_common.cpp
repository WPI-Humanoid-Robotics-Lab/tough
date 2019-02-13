#include <tough_control_common/tough_control_common.h>

ToughControlCommon::ToughControlCommon(ros::NodeHandle nh) : nh_(nh), armTraj(nh), chestTraj(nh), pelvisTraj(nh)

{
  std::string robot_name;
  nh.getParam(TOUGH_COMMON_NAMES::ROBOT_NAME_PARAM, robot_name);

  // set the publisher
  stop_traj_pub_ = nh_.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>(
      TOUGH_COMMON_NAMES::TOPIC_PREFIX + robot_name + TOUGH_COMMON_NAMES::CONTROL_TOPIC_PREFIX +
          TOUGH_COMMON_NAMES::STOP_ALL_TRAJECTORY_TOPIC,
      1, true);
}

ToughControlCommon::~ToughControlCommon()
{
}

void ToughControlCommon::stopAllTrajectories(void)
{
  ihmc_msgs::StopAllTrajectoryRosMessage stop_msg;
  stop_msg.unique_id = -1;

  // send the message
  stop_traj_pub_.publish(stop_msg);

  ros::Duration(1).sleep();
}

void ToughControlCommon::resetRobot()
{
  armTraj.moveToDefaultPose(RobotSide::LEFT, 1.0f);
  armTraj.moveToDefaultPose(RobotSide::RIGHT, 1.0f);
  pelvisTraj.controlPelvisHeight(0.75f);
  chestTraj.resetPose(2.0f);
  ros::Duration(2).sleep();
}
