#include <tough_controller_interface/leg_control_interface.h>
#include <tf/transform_listener.h>
#include <tough_common/robot_state.h>

LegControlInterface::LegControlInterface(ros::NodeHandle nh) : ToughControlInterface(nh)
{
  using namespace TOUGH_COMMON_NAMES;
  this->legTrajectoryPub_ =
      nh_.advertise<ihmc_msgs::FootTrajectoryRosMessage>(control_topic_prefix_ + FOOTSTEP_TRAJECTORY_TOPIC, 1);
  this->loadEndEffPub_ =
      nh_.advertise<ihmc_msgs::FootLoadBearingRosMessage>(control_topic_prefix_ + FOOTSTEP_LOAD_BEARING_TOPIC, 1);
}

LegControlInterface::~LegControlInterface()
{
}

void LegControlInterface::moveFoot(const RobotSide side, const std::vector<geometry_msgs::Pose>& foot_goal_poses,
                                   const float time)
{
  ihmc_msgs::FootTrajectoryRosMessage foot;
  initializeFootTrajectoryRosMessage(side, foot);

  foot.taskspace_trajectory_points.resize(foot_goal_poses.size());

  for (size_t i = 0; i < foot_goal_poses.size(); i++)
  {
    foot.taskspace_trajectory_points[i].position = foot_goal_poses[i].position;
    foot.taskspace_trajectory_points[i].orientation = foot_goal_poses[i].orientation;
    foot.taskspace_trajectory_points[i].unique_id = id_;
    foot.taskspace_trajectory_points[i].time = time / (float)foot_goal_poses.size() * (i + 1);
  }

  legTrajectoryPub_.publish(foot);

  return;
}

void LegControlInterface::moveFoot(const RobotSide side, const geometry_msgs::Pose& foot_goal_pose, const float time)
{
  return moveFoot(side, { foot_goal_pose }, time);
}

void LegControlInterface::raiseLeg(const RobotSide side, const float offset, const float time)
{
  geometry_msgs::Pose goal_pose;
  std::string foot_frame = rd_->getFootFrame(side);

  state_informer_->getCurrentPose(foot_frame, goal_pose);
  goal_pose.position.z += offset;
  return moveFoot(side, goal_pose, time);
}

void LegControlInterface::placeLeg(const RobotSide side, const float offset, const float time)
{
  return raiseLeg(side, -1 * offset, time);
}

void LegControlInterface::curlLeg(RobotSide side, float radius, float time)
{
  ihmc_msgs::FootTrajectoryRosMessage foot;
  initializeFootTrajectoryRosMessage(side, foot);
  std::string foot_frame = rd_->getFootFrame(side);

  geometry_msgs::Pose current, goal;
  state_informer_->getCurrentPose(foot_frame, current, rd_->getPelvisFrame());

  goal.position.x = current.position.x - radius;
  goal.position.y = current.position.y;
  goal.position.z = current.position.z + radius;

  goal.orientation.x = 0;
  goal.orientation.y = 0.5;
  goal.orientation.z = 0;
  goal.orientation.w = 0.866;

  // converting back to world frame
  state_informer_->transformPose(goal, goal, rd_->getPelvisFrame());

  // get current position
  foot.taskspace_trajectory_points.resize(1);
  foot.taskspace_trajectory_points.begin()->position = goal.position;
  foot.taskspace_trajectory_points.begin()->orientation = goal.orientation;
  foot.taskspace_trajectory_points.begin()->unique_id = id_;
  foot.taskspace_trajectory_points.begin()->time = time;

  legTrajectoryPub_.publish(foot);
  return;
}

bool LegControlInterface::getJointSpaceState(std::vector<double>& joints, RobotSide side)
{
  return false;
}

bool LegControlInterface::getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side, std::string fixedFrame)
{
  return state_informer_->getCurrentPose(rd_->getFootFrame(side), pose, fixedFrame);
}
