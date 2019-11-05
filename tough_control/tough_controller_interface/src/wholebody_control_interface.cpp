#include "tough_controller_interface/wholebody_control_interface.h"

WholebodyControlInterface::WholebodyControlInterface(ros::NodeHandle& nh)
  : ToughControlInterface(nh), chestController_(nh), armController_(nh)
{
  m_wholebodyPub = nh_.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>(
      control_topic_prefix_ + TOUGH_COMMON_NAMES::WHOLEBODY_TRAJECTORY_TOPIC, 10, true);

  rd_->getLeftArmJointNames(left_arm_joint_names_);
  rd_->getRightArmJointNames(right_arm_joint_names_);
  rd_->getChestJointNames(chest_joint_names_);

  chest_start_index_ = state_informer_->getJointNumber(chest_joint_names_.front());
  left_arm_start_index_ = state_informer_->getJointNumber(left_arm_joint_names_.front());
  right_arm_start_index_ = state_informer_->getJointNumber(right_arm_joint_names_.front());

  state_informer_->getJointNames(joint_names_);
}

bool WholebodyControlInterface::getJointSpaceState(std::vector<double>& joints, RobotSide side)
{
  joints.clear();
  state_informer_->getJointPositions(joints);

  return !joints.empty();
}

bool WholebodyControlInterface::getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side, std::string fixedFrame)
{
  return state_informer_->getCurrentPose(rd_->getPelvisFrame(), pose, fixedFrame);
}

void WholebodyControlInterface::executeTrajectory(const moveit_msgs::RobotTrajectory& traj)
{
  return executeTrajectory(traj.joint_trajectory);
}

void WholebodyControlInterface::executeTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
  ihmc_msgs::WholeBodyTrajectoryRosMessage wholeBodyMsg;

  initializeWholebodyMessage(wholeBodyMsg);
  parseTrajectory(traj, wholeBodyMsg);
  m_wholebodyPub.publish(wholeBodyMsg);
  ros::Duration(0.1).sleep();
}

void WholebodyControlInterface::initializeWholebodyMessage(ihmc_msgs::WholeBodyTrajectoryRosMessage& wholeBodyMsg)
{
  // Setting seed for random number generator.
  // More details:
  // https://stackoverflow.com/questions/9459035/why-does-rand-yield-the-same-sequence-of-numbers-on-every-run
  srand(time(NULL));

  // Setting unique id non zero for messages to be used
  wholeBodyMsg.unique_id = rand() % 100 + 1;

  // setting default values for empty messages
  wholeBodyMsg.left_arm_trajectory_message.robot_side = LEFT;
  wholeBodyMsg.right_arm_trajectory_message.robot_side = RIGHT;

  wholeBodyMsg.left_foot_trajectory_message.robot_side = LEFT;
  wholeBodyMsg.right_foot_trajectory_message.robot_side = RIGHT;

  wholeBodyMsg.left_hand_trajectory_message.robot_side = LEFT;
  wholeBodyMsg.right_hand_trajectory_message.robot_side = RIGHT;

  // Specifying execution modes
  wholeBodyMsg.chest_trajectory_message.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  wholeBodyMsg.right_arm_trajectory_message.execution_mode = ihmc_msgs::ArmTrajectoryRosMessage::OVERRIDE;
  wholeBodyMsg.left_arm_trajectory_message.execution_mode = ihmc_msgs::ArmTrajectoryRosMessage::OVERRIDE;
  wholeBodyMsg.left_foot_trajectory_message.execution_mode = ihmc_msgs::FootTrajectoryRosMessage::OVERRIDE;
  wholeBodyMsg.right_foot_trajectory_message.execution_mode = ihmc_msgs::FootTrajectoryRosMessage::OVERRIDE;
  wholeBodyMsg.left_hand_trajectory_message.execution_mode = ihmc_msgs::HandTrajectoryRosMessage::OVERRIDE;
  wholeBodyMsg.right_hand_trajectory_message.execution_mode = ihmc_msgs::HandTrajectoryRosMessage::OVERRIDE;

  // unique ID will be updated if there is a valid trajectory point available
  wholeBodyMsg.chest_trajectory_message.unique_id = rand() % 100 + 1;
  wholeBodyMsg.right_arm_trajectory_message.unique_id = rand() % 100 + 1;
  wholeBodyMsg.left_arm_trajectory_message.unique_id = rand() % 100 + 1;

  // taskspace trajectories
  wholeBodyMsg.left_foot_trajectory_message.unique_id = 0;
  wholeBodyMsg.right_foot_trajectory_message.unique_id = 0;
  wholeBodyMsg.left_hand_trajectory_message.unique_id = 0;
  wholeBodyMsg.right_hand_trajectory_message.unique_id = 0;

  ihmc_msgs::FrameInformationRosMessage frameInfo;
  frameInfo.data_reference_frame_id = rd_->getPelvisZUPFrameHash();
  frameInfo.trajectory_reference_frame_id = rd_->getPelvisZUPFrameHash();

  wholeBodyMsg.chest_trajectory_message.frame_information = frameInfo;
  wholeBodyMsg.left_foot_trajectory_message.frame_information = frameInfo;
  wholeBodyMsg.right_foot_trajectory_message.frame_information = frameInfo;
  wholeBodyMsg.pelvis_trajectory_message.frame_information = frameInfo;
  wholeBodyMsg.left_hand_trajectory_message.frame_information = frameInfo;
  wholeBodyMsg.right_hand_trajectory_message.frame_information = frameInfo;

  chestController_.setupFrameAndMode(wholeBodyMsg.chest_trajectory_message);
  armController_.setupArmMessage(RobotSide::LEFT, wholeBodyMsg.left_arm_trajectory_message);
  armController_.setupArmMessage(RobotSide::RIGHT, wholeBodyMsg.right_arm_trajectory_message);
}

void WholebodyControlInterface::parseTrajectory(const trajectory_msgs::JointTrajectory& traj,
                                                ihmc_msgs::WholeBodyTrajectoryRosMessage& wholeBodyMsg)
{
  joint_positions_.resize(0);
  state_informer_->getJointPositions(joint_positions_);

  double traj_point_time = 0.0;

  for (auto& traj_pts : traj.points)
  {
    traj_point_time = traj_pts.time_from_start.toSec();

    for (int i = 0; i < traj.joint_names.size(); i++)
    {
      int index = state_informer_->getJointNumber(traj.joint_names.at(i));
      joint_positions_.at(index) = traj_pts.positions.at(i);
    }

    // CHEST TRAJECTORY
    geometry_msgs::Quaternion quat;
    quat = getChestQuaternion(chest_start_index_, joint_positions_);
    chestController_.appendChestTrajectoryPoint(quat, wholeBodyMsg.chest_trajectory_message, traj_point_time);

    // LEFT ARM TRAJECTORY
    std::vector<double> positions;
    positions.assign(joint_positions_.begin() + left_arm_start_index_,
                     joint_positions_.begin() + left_arm_start_index_ + left_arm_joint_names_.size());
    armController_.appendTrajectoryPoint(wholeBodyMsg.left_arm_trajectory_message, traj_point_time, positions);

    // RIGHT ARM TRAJECTORY
    positions.clear();
    positions.assign(joint_positions_.begin() + right_arm_start_index_,
                     joint_positions_.begin() + right_arm_start_index_ + right_arm_joint_names_.size());
    armController_.appendTrajectoryPoint(wholeBodyMsg.right_arm_trajectory_message, traj_point_time, positions);
  }
}

void WholebodyControlInterface::executeAccnTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
  for (auto& traj_pts : traj.points)
  {
    chest_acceleration_.resize(0);
    left_arm_acceleration_.resize(0);
    right_arm_acceleration_.resize(0);
    
    joint_acceleration_.resize(joint_names_.size());
    std::fill(joint_acceleration_.begin(), joint_acceleration_.end(), 0);

    for (int i = 0; i < traj.joint_names.size(); i++)
    {
      int index = state_informer_->getJointNumber(traj.joint_names.at(i));
      joint_acceleration_.at(index) = traj_pts.accelerations.at(i);
    }

    // CHEST TRAJECTORY
    chest_acceleration_.insert(chest_acceleration_.begin(), joint_acceleration_.begin() + chest_start_index_,
                               joint_acceleration_.begin() + chest_start_index_ + chest_joint_names_.size());
    // chestController_.executeChestAccelerations(chest_acceleration_);

    // LEFT ARM TRAJECTORY
    left_arm_acceleration_.insert(left_arm_acceleration_.begin(), joint_acceleration_.begin() + left_arm_start_index_,
                                  joint_acceleration_.begin() + left_arm_start_index_ + left_arm_joint_names_.size());
    armController_.moveArmJointsAcceleration(RobotSide::LEFT, left_arm_acceleration_);
    ros::Duration(0.01).sleep();

    // RIGHT ARM TRAJECTORY
    right_arm_acceleration_.insert(
        right_arm_acceleration_.begin(), joint_acceleration_.begin() + right_arm_start_index_,
        joint_acceleration_.begin() + right_arm_start_index_ + right_arm_joint_names_.size());
    armController_.moveArmJointsAcceleration(RobotSide::RIGHT, right_arm_acceleration_);
    ros::Duration(0.01).sleep();
  }
}