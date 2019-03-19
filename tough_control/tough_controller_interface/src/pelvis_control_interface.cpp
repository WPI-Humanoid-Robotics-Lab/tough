#include <tough_controller_interface/pelvis_control_interface.h>

PelvisControlInterface::PelvisControlInterface(ros::NodeHandle nh) : ToughControlInterface(nh)
{
  pelvisHeightPublisher_ = nh_.advertise<ihmc_msgs::PelvisHeightTrajectoryRosMessage>(
      control_topic_prefix_ + TOUGH_COMMON_NAMES::PELVIS_HEIGHT_TRAJECTORY_TOPIC, 1, true);

  homePositionPublisher_ =
      nh_.advertise<ihmc_msgs::GoHomeRosMessage>(control_topic_prefix_ + TOUGH_COMMON_NAMES::GO_HOME_TOPIC, 1, true);
}

PelvisControlInterface::~PelvisControlInterface()
{
  pelvisHeightPublisher_.shutdown();
}

/**
 * @brief PelvisControlInterface::controlPelvisHeight controls the height of the pelvis with respect to the feet
 * @param height is the height in meters
 */
void PelvisControlInterface::controlPelvisHeight(float height, float duration)
{
  ihmc_msgs::PelvisHeightTrajectoryRosMessage msg;
  ihmc_msgs::EuclideanTrajectoryPointRosMessage p;

  geometry_msgs::Pose foot_pose;
  state_informer_->getCurrentPose(rd_->getLeftFootFrameName(), foot_pose);

  ihmc_msgs::FrameInformationRosMessage reference_frame;
  reference_frame.trajectory_reference_frame_id = rd_->getPelvisZUPFrameHash();  // Pelvis frame
  reference_frame.data_reference_frame_id = rd_->getPelvisZUPFrameHash();        // Pelvis frame
  msg.frame_information = reference_frame;
  p.position.z = height + foot_pose.position.z - rd_->getFootFrameOffset();
  p.time = duration;

  msg.taskspace_trajectory_points.clear();
  msg.taskspace_trajectory_points.push_back(p);
  msg.use_custom_control_frame = false;

  msg.unique_id = id_++;

  // publish the message
  publishPelvisMessage(msg);
}

void PelvisControlInterface::publishPelvisMessage(const ihmc_msgs::PelvisHeightTrajectoryRosMessage& msg) const
{
  this->pelvisHeightPublisher_.publish(msg);
}

void PelvisControlInterface::resetPose(float time)
{
  ihmc_msgs::GoHomeRosMessage go_home;
  go_home.body_part = ihmc_msgs::GoHomeRosMessage::PELVIS;

  go_home.trajectory_time = time;
  go_home.unique_id = PelvisControlInterface::id_++;

  homePositionPublisher_.publish(go_home);
}

bool PelvisControlInterface::getJointSpaceState(std::vector<double>& joints, RobotSide side)
{
  return false;
}

bool PelvisControlInterface::getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side, std::string fixedFrame)
{
  return state_informer_->getCurrentPose(rd_->getPelvisFrame(), pose, fixedFrame);
}
