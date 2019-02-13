#include <tough_controller_interface/head_control_interface.h>
#include <tf/transform_listener.h>
#include <tough_common/robot_state.h>

HeadControlInterface::HeadControlInterface(ros::NodeHandle nh) : ToughControllerInterface(nh)
{
  neckTrajPublisher = nh_.advertise<ihmc_msgs::NeckTrajectoryRosMessage>(
      control_topic_prefix_ + TOUGH_COMMON_NAMES::NECK_TRAJECTORY_TOPIC, 1, true);
  headTrajPublisher = nh_.advertise<ihmc_msgs::HeadTrajectoryRosMessage>(
      control_topic_prefix_ + TOUGH_COMMON_NAMES::HEAD_TRAJECTORY_TOPIC, 1, true);
  NUM_NECK_JOINTS = rd_->getNumberOfNeckJoints();
}

HeadControlInterface::~HeadControlInterface()
{
}

void HeadControlInterface::appendNeckTrajectoryPoint(ihmc_msgs::NeckTrajectoryRosMessage& msg, float time,
                                                     std::vector<float> pos)
{
  for (int i = 0; i < NUM_NECK_JOINTS; i++)
  {
    ihmc_msgs::TrajectoryPoint1DRosMessage p;
    ihmc_msgs::OneDoFJointTrajectoryRosMessage t;
    t.trajectory_points.clear();

    p.time = time;
    p.position = pos[i];
    p.unique_id = HeadControlInterface::id_;
    t.trajectory_points.push_back(p);
    t.unique_id = HeadControlInterface::id_;
    msg.joint_trajectory_messages.push_back(t);
  }
}

void HeadControlInterface::moveHead(const float roll, const float pitch, const float yaw, const float time)
{
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion quaternion;
  tf::quaternionTFToMsg(q, quaternion);
  moveHead(quaternion, time);
}

void HeadControlInterface::moveHead(const geometry_msgs::Quaternion& quaternion, const float time)
{
  ihmc_msgs::HeadTrajectoryRosMessage msg;
  ihmc_msgs::SO3TrajectoryPointRosMessage data;
  ihmc_msgs::FrameInformationRosMessage reference_frame;

  reference_frame.trajectory_reference_frame_id = rd_->getPelvisZUPFrameHash();  // Pelvis frame
  reference_frame.data_reference_frame_id = rd_->getPelvisZUPFrameHash();        // Pelvis frame
  msg.frame_information = reference_frame;

  data.orientation = quaternion;

  HeadControlInterface::id_++;
  msg.unique_id = HeadControlInterface::id_;
  msg.execution_mode = msg.OVERRIDE;

  msg.taskspace_trajectory_points.clear();

  msg.taskspace_trajectory_points.push_back(data);

  // publish the message
  headTrajPublisher.publish(msg);
}

void HeadControlInterface::moveHead(const std::vector<std::vector<float> >& trajectory_points, const float time)
{
  ihmc_msgs::HeadTrajectoryRosMessage msg;
  ihmc_msgs::FrameInformationRosMessage reference_frame;

  reference_frame.trajectory_reference_frame_id = rd_->getPelvisZUPFrameHash();  // Pelvis frame
  reference_frame.data_reference_frame_id = rd_->getPelvisZUPFrameHash();        // Pelvis frame
  msg.frame_information = reference_frame;

  HeadControlInterface::id_++;
  msg.unique_id = HeadControlInterface::id_;
  msg.execution_mode = msg.OVERRIDE;

  msg.taskspace_trajectory_points.clear();

  for (int i = 0; i < trajectory_points.size(); i++)
  {
    ihmc_msgs::SO3TrajectoryPointRosMessage data;

    float roll = trajectory_points[i][0];
    float pitch = trajectory_points[i][1];
    float yaw = trajectory_points[i][2];

    data.time = time;
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(q, quat);
    data.orientation = quat;

    msg.taskspace_trajectory_points.push_back(data);
  }
  // publish the message
  headTrajPublisher.publish(msg);
}

void HeadControlInterface::moveNeckJoints(const std::vector<std::vector<float> >& neck_pose, const float time)
{
  ihmc_msgs::NeckTrajectoryRosMessage msg;

  HeadControlInterface::id_++;
  msg.unique_id = HeadControlInterface::id_;

  // Add all neck trajectory points to the trajectory message
  for (int i = 0; i < neck_pose.size(); i++)
  {
    if (neck_pose[i].size() != NUM_NECK_JOINTS)
      ROS_ERROR("Check number of trajectory points");
    appendNeckTrajectoryPoint(msg, time / neck_pose.size(), neck_pose[i]);
  }
  // publish the message
  neckTrajPublisher.publish(msg);
}

bool HeadControlInterface::getJointSpaceState(std::vector<double>& joints, RobotSide side)
{
  return false;
}

bool HeadControlInterface::getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side, std::string fixedFrame)
{
  return state_informer_->getCurrentPose(TOUGH_COMMON_NAMES::ROBOT_HEAD_FRAME_TF, pose, fixedFrame);
}

int HeadControlInterface::getNumNeckJoints() const
{
  return NUM_NECK_JOINTS;
}
