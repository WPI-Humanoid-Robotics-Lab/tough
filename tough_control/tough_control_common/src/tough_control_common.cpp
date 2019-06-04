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

void ToughControlCommon::stopAllTrajectories()
{
  ihmc_msgs::StopAllTrajectoryRosMessage stop_msg;
  stop_msg.unique_id = -1;

  // send the message
  stop_traj_pub_.publish(stop_msg);

  ros::Duration(0.1).sleep();
}

void ToughControlCommon::resetRobot(float time, bool blockCall)
{
  armTraj.moveToDefaultPose(RobotSide::LEFT, time);
  armTraj.moveToDefaultPose(RobotSide::RIGHT, time);
  pelvisTraj.resetPose(time);
  chestTraj.resetPose(time);
  if (blockCall)
  {
    ros::Duration(time).sleep();
  }
}
