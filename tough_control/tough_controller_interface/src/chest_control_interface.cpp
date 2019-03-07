#include <tough_controller_interface/chest_control_interface.h>
#include <tf/transform_listener.h>

ChestControlInterface::ChestControlInterface(ros::NodeHandle nh) : ToughControlInterface(nh)
{
  chestTrajPublisher_ = nh_.advertise<ihmc_msgs::ChestTrajectoryRosMessage>(
      control_topic_prefix_ + TOUGH_COMMON_NAMES::CHEST_TRAJECTORY_TOPIC, 1, true);

  homePositionPublisher_ =
      nh_.advertise<ihmc_msgs::GoHomeRosMessage>(control_topic_prefix_ + TOUGH_COMMON_NAMES::GO_HOME_TOPIC, 1, true);
}

ChestControlInterface::~ChestControlInterface()
{
}

void ChestControlInterface::controlChest(const float roll, const float pitch, const float yaw, const float time,
                                         const int execution_mode)
{
  tf::Quaternion quatInPelvisFrame;
  quatInPelvisFrame.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion quat;
  tf::quaternionTFToMsg(quatInPelvisFrame, quat);

  controlChest(quat, time, execution_mode);
}

void ChestControlInterface::controlChest(const geometry_msgs::Quaternion quat, const float time, int execution_mode)
{
  ihmc_msgs::ChestTrajectoryRosMessage msg;
  generateMessage(quat, time, execution_mode, msg);

  // publish the message
  chestTrajPublisher_.publish(msg);
}

void ChestControlInterface::executeMessage(const ihmc_msgs::ChestTrajectoryRosMessage& msg)
{
  chestTrajPublisher_.publish(msg);
}

void ChestControlInterface::setupFrameAndMode(ihmc_msgs::ChestTrajectoryRosMessage& msg, const int mode,
                                              const int frame_hash)
{
  msg.unique_id = ChestControlInterface::id_++;
  msg.execution_mode = mode;

  ihmc_msgs::FrameInformationRosMessage reference_frame;
  reference_frame.trajectory_reference_frame_id = frame_hash;
  reference_frame.data_reference_frame_id = frame_hash;
  msg.frame_information = reference_frame;
}

void ChestControlInterface::appendChestTrajectoryPoint(const geometry_msgs::Quaternion q_in,
                                                       ihmc_msgs::ChestTrajectoryRosMessage& msg, const double time)
{
  ihmc_msgs::SO3TrajectoryPointRosMessage point;
  point.orientation.x = q_in.x;
  point.orientation.y = q_in.y;
  point.orientation.z = q_in.z;
  point.orientation.w = q_in.w;

  point.time = time;
  msg.taskspace_trajectory_points.push_back(point);
}

void ChestControlInterface::generateMessage(const geometry_msgs::Quaternion& quat, const float time,
                                            const int execution_mode, ihmc_msgs::ChestTrajectoryRosMessage& msg)
{
  setupFrameAndMode(msg, execution_mode);
  appendChestTrajectoryPoint(quat, msg, time);
}

void ChestControlInterface::generateMessage(
    const std::vector<ihmc_msgs::SO3TrajectoryPointRosMessage>& chest_trajectory, const int execution_mode,
    ihmc_msgs::ChestTrajectoryRosMessage& msg)
{
  msg.unique_id = ChestControlInterface::id_++;
  msg.execution_mode = execution_mode;

  ihmc_msgs::FrameInformationRosMessage reference_frame;
  reference_frame.trajectory_reference_frame_id = rd_->getPelvisZUPFrameHash();  // Pelvis frame
  reference_frame.data_reference_frame_id = rd_->getPelvisZUPFrameHash();        // Pelvis frame

  msg.frame_information = reference_frame;
  msg.taskspace_trajectory_points.resize(chest_trajectory.size());
  msg.taskspace_trajectory_points = chest_trajectory;
}

void ChestControlInterface::getChestOrientation(geometry_msgs::Quaternion& orientation)
{
  geometry_msgs::Pose chest_pose;
  state_informer_->getCurrentPose(rd_->getTorsoFrame(), chest_pose, rd_->getPelvisFrame());
  orientation = chest_pose.orientation;
}

void ChestControlInterface::resetPose(float time)
{
  ihmc_msgs::GoHomeRosMessage go_home;
  go_home.body_part = ihmc_msgs::GoHomeRosMessage::CHEST;
  go_home.trajectory_time = time;
  go_home.unique_id = ChestControlInterface::id_++;
  homePositionPublisher_.publish(go_home);
  ros::Duration(0.5).sleep();
}

bool ChestControlInterface::getJointSpaceState(std::vector<double>& joints, RobotSide side)
{
  std::vector<std::string> chest_joint_names;
  rd_->getChestJointNames(chest_joint_names);

  for (auto joint : chest_joint_names)
  {
    joints.push_back(state_informer_->getJointPosition(joint));
  }
}

bool ChestControlInterface::getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side, std::string fixedFrame)
{
  return state_informer_->getCurrentPose(rd_->getTorsoFrame(), pose, fixedFrame);
}
