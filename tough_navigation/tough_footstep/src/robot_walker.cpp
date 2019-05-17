#include "tough_footstep/robot_walker.h"
#include "tough_common/tough_common_names.h"
#include <iostream>
#include <ros/ros.h>

int RobotWalker::id = 1;

RobotWalker::RobotWalker(ros::NodeHandle nh, double InTransferTime, double InSwingTime, int InMode, double swingHeight)
  : nh_(nh)
{
  using namespace TOUGH_COMMON_NAMES;
  current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
  rd_ = RobotDescription::getRobotDescription(nh_);
  const std::string robot_name = rd_->getRobotName();
  const std::string control_prefix = TOPIC_PREFIX + robot_name + CONTROL_TOPIC_PREFIX;
  const std::string output_prefix = TOPIC_PREFIX + robot_name + OUTPUT_TOPIC_PREFIX;

  this->footsteps_pub_ =
      nh_.advertise<ihmc_msgs::FootstepDataListRosMessage>(control_prefix + FOOTSTEP_LIST_TOPIC, 1, true);
  this->nudgestep_pub_ =
      nh_.advertise<ihmc_msgs::FootTrajectoryRosMessage>(control_prefix + FOOTSTEP_TRAJECTORY_TOPIC, 1, true);
  this->loadeff_pub =
      nh_.advertise<ihmc_msgs::FootLoadBearingRosMessage>(control_prefix + FOOTSTEP_LOAD_BEARING_TOPIC, 1, true);
  this->abort_footsteps_pub_ =
      nh_.advertise<ihmc_msgs::AbortWalkingRosMessage>(control_prefix + ABORT_WALKING_TOPIC, 1, true);
  this->footstep_status_ =
      nh_.subscribe(output_prefix + FOOTSTEP_STATUS_TOPIC, 5, &RobotWalker::footstepStatusCB, this);
  this->walking_status_ =
      nh_.subscribe(output_prefix + WALKING_STATUS_TOPIC, 0, &RobotWalker::walkingStatusCB, this);

  transfer_time_ = InTransferTime;
  swing_time_ = InSwingTime;
  execution_mode_ = InMode;
  swing_height_ = swingHeight;
  previous_message_id_ = 0;
  isWalking = false;
  ros::Duration(0.5).sleep();
  step_counter_ = 0;

  right_foot_frame_.data = rd_->getRightFootFrameName();
  left_foot_frame_.data = rd_->getLeftFootFrameName();

  /* This timer is used for waiting till the steps are executed. When the robot starts walking, the timer is reset and
   * at every step it resets. If the timer crosses 5 seconds without any steps, there could be some hardware error.
   */
  cbTime_ = ros::Time::now();
}

/**
 * @brief RobotWalker::~RobotWalker
 */
RobotWalker::~RobotWalker()
{
}

/**
 * @brief RobotWalker::footstepStatusCB is a callback function that updates footstep status
 * @param msg
 */
void RobotWalker::footstepStatusCB(const ihmc_msgs::FootstepStatusRosMessage& msg)
{
  if (msg.status == ihmc_msgs::FootstepStatusRosMessage::COMPLETED)
  {
    step_counter_++;
  }

  // reset the timer
  cbTime_ = ros::Time::now();

  return;
}

void RobotWalker::walkingStatusCB(const ihmc_msgs::WalkingStatusRosMessage& msg) {
  isWalking = msg.status == ihmc_msgs::WalkingStatusRosMessage::STARTED;
}
// calls the footstep planner to plan path and walks to a 2D goal.
bool RobotWalker::walkToGoal(const geometry_msgs::Pose2D& goal, bool waitForSteps)
{
  ihmc_msgs::FootstepDataListRosMessage list;
  initializeFootstepDataListRosMessage(list);
  if (this->getFootstep(goal, list))
  {
    this->footsteps_pub_.publish(list);
    previous_message_id_ = RobotWalker::id++;

    if (waitForSteps)
    {
      cbTime_ = ros::Time::now();
      this->waitForSteps(list.footstep_data_list.size());
    }
    return true;
  }
  ROS_WARN("Failed to plan footsteps. Does map exist? Is the footstep planner on?");
  return false;
}

void RobotWalker::stepAtPose(const geometry_msgs::Pose& goal, const RobotSide side, bool waitForSteps,
                             const bool queue, const std::string refFrame)
{
  if(queue && isWalking) {
    this->execution_mode_ = ihmc_msgs::FootstepDataListRosMessage::QUEUE;
  }
  ihmc_msgs::FootstepDataListRosMessage list;
  initializeFootstepDataListRosMessage(list);
  
  if (queue)
  {
    this->execution_mode_ = ihmc_msgs::FootstepDataListRosMessage::OVERRIDE;
  }
  geometry_msgs::Pose goalInWorld;
  current_state_->transformPose(goal, goalInWorld, refFrame);
  list.footstep_data_list.push_back(*getOffsetStep(side, goalInWorld));

  this->footsteps_pub_.publish(list);
  previous_message_id_ = RobotWalker::id++;

  if (waitForSteps)
  {
    cbTime_ = ros::Time::now();
    this->waitForSteps(list.footstep_data_list.size());
  }
  return;
}

// walks certain number of defined footsteps. steps defined wrt world frame.
bool RobotWalker::walkNSteps(const int numSteps, const float xOffset, float yOffset, RobotSide startLeg,
                             bool waitForSteps)
{
  ihmc_msgs::FootstepDataListRosMessage list;
  initializeFootstepDataListRosMessage(list);
  RobotSide side = startLeg;

  for (int m = 1; m <= numSteps; m++)
  {
    list.footstep_data_list.push_back(*getOffsetStep(side, m * xOffset, m * yOffset));
    side = (RobotSide)!side;
  }
  list.footstep_data_list.push_back(*getOffsetStep(side, numSteps * xOffset, numSteps * yOffset));

  this->walkGivenSteps(list, waitForSteps);
  return true;
}

// walks certain number of defined footsteps. steps defined wrt pelvis frame.
bool RobotWalker::walkNStepsWRTPelvis(const int numSteps, const float xOffset, float yOffset, RobotSide startLeg,
                                      bool waitForSteps)
{
  ihmc_msgs::FootstepDataListRosMessage list;
  initializeFootstepDataListRosMessage(list);

  RobotSide side = startLeg;
  for (int m = 1; m <= numSteps; m++)
  {
    list.footstep_data_list.push_back(*getOffsetStepWRTPelvis(side, m * xOffset, m * yOffset));
    side = (RobotSide)!side;
  }

  list.footstep_data_list.push_back(*getOffsetStepWRTPelvis(side, numSteps * xOffset, numSteps * yOffset));

  this->walkGivenSteps(list, waitForSteps);
  return true;
}

// walks predefined steps which could have varying step length and step widths. This is defined wrt World frame.
bool RobotWalker::walkPreComputedSteps(const std::vector<float>& xOffset, const std::vector<float>& yOffset,
                                       const RobotSide startLeg)
{
  ihmc_msgs::FootstepDataListRosMessage list;
  initializeFootstepDataListRosMessage(list);

  if (xOffset.size() != yOffset.size())
  {
    ROS_ERROR("X Offset and Y Offset have different size");
    return false;
  }

  size_t numberOfSteps = xOffset.size();
  RobotSide side = startLeg;
  for (int m = 1; m <= numberOfSteps; m++)
  {
    list.footstep_data_list.push_back(*getOffsetStep(side, xOffset.at(m - 1), yOffset.at(m - 1)));
    side = (RobotSide)!side;
  }

  this->walkGivenSteps(list);
  return true;
}

// walks predefined steps which could have varying step length and step widths. This is defined wrt Pelvis frame.
bool RobotWalker::walkLocalPreComputedSteps(const std::vector<float>& xOffset, const std::vector<float>& yOffset,
                                            const RobotSide startLeg)
{
  ihmc_msgs::FootstepDataListRosMessage list;
  initializeFootstepDataListRosMessage(list);

  if (xOffset.size() != yOffset.size())
    ROS_ERROR("X Offset and Y Offset have different size");

  ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
  ihmc_msgs::FootstepDataRosMessage::Ptr newFootStep(new ihmc_msgs::FootstepDataRosMessage());

  geometry_msgs::Point currentWorldLocation, currentPelvisLocation;

  size_t numberOfSteps = xOffset.size();
  RobotSide side = startLeg;

  for (int m = 1; m <= numberOfSteps; ++m)
  {
    getCurrentStep(side, *current);
    side = (RobotSide)!side;

    currentWorldLocation = current->location;
    current_state_->transformPoint(currentWorldLocation, currentPelvisLocation, TOUGH_COMMON_NAMES::WORLD_TF,
                                   rd_->getPelvisFrame());

    currentPelvisLocation.x += xOffset.at(m - 1);
    currentPelvisLocation.y += yOffset.at(m - 1);
    current_state_->transformPoint(currentPelvisLocation, currentWorldLocation, rd_->getPelvisFrame(),
                                   TOUGH_COMMON_NAMES::WORLD_TF);
    newFootStep->location = currentWorldLocation;
    newFootStep->location.z = current->location.z;
    newFootStep->orientation = current->orientation;
    newFootStep->robot_side = current->robot_side;
    newFootStep->trajectory_type = current->trajectory_type;
    list.footstep_data_list.push_back(*newFootStep);
  }

  this->walkGivenSteps(list);
  return true;
}

bool RobotWalker::walkGivenSteps(const ihmc_msgs::FootstepDataListRosMessage& list, const bool waitForSteps)
{
  this->footsteps_pub_.publish(list);
  this->previous_message_id_ = RobotWalker::id++;
  if (waitForSteps)
  {
    cbTime_ = ros::Time::now();
    this->waitForSteps(list.footstep_data_list.size());
  }
  return true;
}

bool RobotWalker::raiseLeg(RobotSide side, float height, float time)
{
  ihmc_msgs::FootTrajectoryRosMessage foot;
  initializeFootTrajectoryRosMessage(side, foot);

  ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
  getCurrentStep(side, *current);

  // get current position
  foot.taskspace_trajectory_points.begin()->position = current->location;
  foot.taskspace_trajectory_points.begin()->orientation = current->orientation;
  foot.taskspace_trajectory_points.begin()->position.z += height;
  foot.taskspace_trajectory_points.begin()->unique_id = id++;
  foot.taskspace_trajectory_points.begin()->time = time;

  nudgestep_pub_.publish(foot);

  return true;
}

bool RobotWalker::nudgeFoot(const RobotSide side, const float x_offset, const float y_offset,
                            const geometry_msgs::Quaternion* orientation, const float time)
{
  ihmc_msgs::FootTrajectoryRosMessage foot;
  initializeFootTrajectoryRosMessage(side, foot);

  ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
  getCurrentStep(side, *current);

  geometry_msgs::PointStamped pt_in, pt_out;
  pt_in.point = current->location;
  pt_in.header.frame_id = TOUGH_COMMON_NAMES::WORLD_TF;
  current_state_->transformPoint(pt_in, pt_out, rd_->getPelvisFrame());

  pt_out.point.x += x_offset;
  pt_out.point.y += y_offset;
  pt_out.point.z += rd_->getFootFrameOffset();
  // convert back to world frame
  current_state_->transformPoint(pt_out, pt_out);

  // add to data
  foot.taskspace_trajectory_points.begin()->position = pt_out.point;
  if (orientation)
  {
    ROS_INFO_STREAM("Foot orientation" << *orientation);
    foot.taskspace_trajectory_points.begin()->orientation = *orientation;
  }
  else
  {
    foot.taskspace_trajectory_points.begin()->orientation = current->orientation;
  }

  std::cout << "point x" << foot.taskspace_trajectory_points.begin()->position.x << "\n";
  std::cout << "point y" << foot.taskspace_trajectory_points.begin()->position.y << "\n";
  std::cout << "point z" << foot.taskspace_trajectory_points.begin()->position.z << "\n";

  foot.taskspace_trajectory_points.begin()->unique_id = id;
  foot.taskspace_trajectory_points.begin()->time = time;

  nudgestep_pub_.publish(foot);

  return true;
}

bool RobotWalker::moveFoot(const RobotSide side, const std::vector<geometry_msgs::Pose>& foot_goal_poses,
                           const float time)
{
  ihmc_msgs::FootTrajectoryRosMessage foot;
  initializeFootTrajectoryRosMessage(side, foot);

  ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
  getCurrentStep(side, *current);

  foot.taskspace_trajectory_points.resize(foot_goal_poses.size());

  for (size_t i = 0; i < foot_goal_poses.size(); i++)
  {
    foot.taskspace_trajectory_points[i].position = foot_goal_poses[i].position;
    foot.taskspace_trajectory_points[i].orientation = foot_goal_poses[i].orientation;
    foot.taskspace_trajectory_points[i].unique_id = id;
    foot.taskspace_trajectory_points[i].time = time / (float)foot_goal_poses.size() * (i + 1);
  }

  nudgestep_pub_.publish(foot);

  return true;
}

bool RobotWalker::moveFoot(const RobotSide side, const geometry_msgs::Pose& foot_goal_pose, const float time)
{
  std::vector<geometry_msgs::Pose> temp = { foot_goal_pose };
  return moveFoot(side, temp, time);
}

bool RobotWalker::curlLeg(RobotSide side, float radius, float time)
{
  ihmc_msgs::FootTrajectoryRosMessage foot;
  initializeFootTrajectoryRosMessage(side, foot);

  ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
  getCurrentStep(side, *current);

  // converting point in pelvis frame
  geometry_msgs::PointStamped pt_in, pt_out;
  geometry_msgs::Pose final;
  pt_in.point = current->location;
  pt_in.header.frame_id = TOUGH_COMMON_NAMES::WORLD_TF;
  current_state_->transformPoint(pt_in, pt_out, rd_->getPelvisFrame());

  final.position.x = pt_out.point.x - radius;
  final.position.y = pt_out.point.y;
  final.position.z = pt_out.point.z + radius;

  final.orientation.x = 0;
  final.orientation.y = 0.5;
  final.orientation.z = 0;
  final.orientation.w = 0.866;

  // converting back to world frame
  current_state_->transformPose(final, final, rd_->getPelvisFrame());

  // get current position
  foot.taskspace_trajectory_points.begin()->position = final.position;
  foot.taskspace_trajectory_points.begin()->orientation = final.orientation;
  foot.taskspace_trajectory_points.begin()->unique_id = id;
  foot.taskspace_trajectory_points.begin()->time = time;

  nudgestep_pub_.publish(foot);
  return true;
}

bool RobotWalker::placeLeg(RobotSide side, float offset, float time)
{
  return raiseLeg(side, -1 * offset, time);
}

// Calls the footstep planner service to get footsteps to reach goal
bool RobotWalker::getFootstep(const geometry_msgs::Pose2D& goal, ihmc_msgs::FootstepDataListRosMessage& list)
{
  /// @todo fix the robot pose, if the legs are not together before walking.

  geometry_msgs::Pose2D start;
  humanoid_nav_msgs::PlanFootsteps srv;
  footstep_client_ = nh_.serviceClient<humanoid_nav_msgs::PlanFootsteps>(TOUGH_COMMON_NAMES::FOOTSTEP_PLANNER_SERVICE);
  // get start from robot position

  //    ihmc_msgs::FootstepDataRosMessage::Ptr startstep(new ihmc_msgs::FootstepDataRosMessage());
  // this->getCurrentStep(0,*startstep);
  geometry_msgs::Pose pelvisPose, leftFootPose, rightFootPose;
  current_state_->getCurrentPose(rd_->getLeftFootFrameName(), leftFootPose);
  current_state_->getCurrentPose(rd_->getRightFootFrameName(), rightFootPose);

  start.x = (leftFootPose.position.x + rightFootPose.position.x) / 2.0f;
  start.y = (leftFootPose.position.y + rightFootPose.position.y) /
            2.0f;  // This is required to offset the left foot to get center of the
  //    std::cout<< "Start Position  x = " << start.x << "  y = " << start.y<<std::endl;

  start.theta = tf::getYaw(rightFootPose.orientation);

  srv.request.start = start;
  srv.request.goal = goal;
  // The service calls succeeds everytime. result variable stores the actual result of planning
  if (footstep_client_.call(srv) && srv.response.result)
  {
    for (int i = 0; i < srv.response.footsteps.size(); i++)
    {
      ihmc_msgs::FootstepDataRosMessage::Ptr step(new ihmc_msgs::FootstepDataRosMessage());
      bool side = bool(srv.response.footsteps.at(i).leg);
      side = !side;

      this->getCurrentStep(int(side), *step);

      step->location.x = srv.response.footsteps.at(i).pose.x;
      step->location.y = srv.response.footsteps.at(i).pose.y;
      step->location.z = step->location.z;

      tf::Quaternion t = tf::createQuaternionFromYaw(srv.response.footsteps.at(i).pose.theta);
      ROS_DEBUG("Step x  %d %.2f", i, srv.response.footsteps.at(i).pose.x);
      ROS_DEBUG("Step y  %d %.2f", i, srv.response.footsteps.at(i).pose.y);
      ROS_DEBUG("Side  %d %d", i, int(side));

      step->orientation.w = t.w();
      step->orientation.x = t.x();
      step->orientation.y = t.y();
      step->orientation.z = t.z();

      list.footstep_data_list.push_back(*step);
    }
    return true;
  }
  return false;
}

void RobotWalker::abortWalk()
{
  ihmc_msgs::AbortWalkingRosMessage msg;
  msg.unique_id = id++;
  abort_footsteps_pub_.publish(msg);
}

double RobotWalker::getSwingHeight() const
{
  return swing_height_;
}

// Get starting location of the foot

void RobotWalker::getCurrentStep(int side, ihmc_msgs::FootstepDataRosMessage& foot, const std::string base_frame)
{
  std_msgs::String foot_frame = side == LEFT ? left_foot_frame_ : right_foot_frame_;

  geometry_msgs::Pose footPose;
  current_state_->getCurrentPose(foot_frame.data, footPose, base_frame);
  foot.location = footPose.position;
  foot.location.z -= rd_->getFootFrameOffset();
  foot.orientation = footPose.orientation;
  foot.robot_side = side;
  foot.trajectory_type = ihmc_msgs::FootstepDataRosMessage::DEFAULT;

  return;
}

// gives footstep which are offset from current step (only for straight line)

ihmc_msgs::FootstepDataRosMessage::Ptr RobotWalker::getOffsetStep(int side, float x, float y)
{
  ihmc_msgs::FootstepDataRosMessage::Ptr next(new ihmc_msgs::FootstepDataRosMessage());

  this->getCurrentStep(side, *next);
  next->location.x += x;
  next->location.y += y;
  next->swing_height = swing_height_;
  return next;
}

ihmc_msgs::FootstepDataRosMessage::Ptr RobotWalker::getOffsetStep(const RobotSide side, const geometry_msgs::Pose& goal,
                                                                  const std::string base_frame)
{
  ihmc_msgs::FootstepDataRosMessage::Ptr next(new ihmc_msgs::FootstepDataRosMessage());

  this->getCurrentStep(side, *next, base_frame);
  geometry_msgs::Pose goal_out;
  current_state_->transformPose(goal, goal_out, base_frame);
  next->location = goal_out.position;
  next->orientation = goal_out.orientation;
  next->swing_height = swing_height_;
  return next;
}

ihmc_msgs::FootstepDataRosMessage::Ptr RobotWalker::getOffsetStepWRTPelvis(int side, float x, float y)
{
  ihmc_msgs::FootstepDataRosMessage::Ptr next(new ihmc_msgs::FootstepDataRosMessage());
  geometry_msgs::Pose nextFootPose;

  // get the current step in pelvis frame
  getCurrentStep(side, *next, rd_->getPelvisFrame());

  // add the offsets wrt to pelvis
  nextFootPose.position.x = next->location.x + x;
  nextFootPose.position.y = next->location.y + y;
  nextFootPose.position.z = next->location.z;

  // tranform back the point and quaternion to world coordinates
  current_state_->transformPoint(nextFootPose.position, next->location, rd_->getPelvisFrame(), rd_->getWorldFrame());
  current_state_->transformQuaternion(next->orientation, next->orientation, rd_->getPelvisFrame(),
                                      rd_->getWorldFrame());

  next->swing_height = swing_height_;

  // return it
  return next;
}

void RobotWalker::loadEEF(RobotSide side, EE_LOADING load)
{
  ihmc_msgs::FootLoadBearingRosMessage msg;
  msg.unique_id = 1;
  msg.robot_side = side;
  msg.request = (int)load;  // 0 -load 1 -unload
  loadeff_pub.publish(msg);
}

bool RobotWalker::walkRotate(float angle)
{
  // get current position
  ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
  getCurrentStep(RIGHT, *current);

  geometry_msgs::Pose2D goal;
  goal.x = current->location.x;
  goal.y = current->location.y;
  goal.theta = tf::getYaw(current->orientation) + angle;
  return walkToGoal(goal);
}

bool RobotWalker::climbStair(const std::vector<float>& xOffset, const std::vector<float>& zOffset,
                             const RobotSide startLeg)
{
  // This function was used for SRC. This should be generalized for any application.
  ihmc_msgs::FootstepDataListRosMessage list;
  initializeFootstepDataListRosMessage(list);

  float offset = 0.1;

  if (xOffset.size() != zOffset.size())
    ROS_ERROR("X Offset and Z Offset have different size");

  ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
  ihmc_msgs::FootstepDataRosMessage::Ptr newFootStep(new ihmc_msgs::FootstepDataRosMessage());

  geometry_msgs::Point currentWorldLocation, currentPelvisLocation;

  size_t numberOfSteps = xOffset.size();
  RobotSide side = startLeg;

  for (int m = 1; m <= numberOfSteps; ++m)
  {
    getCurrentStep(side, *current);
    side = (RobotSide)!side;
    currentWorldLocation = current->location;

    current_state_->transformPoint(currentWorldLocation, currentPelvisLocation, TOUGH_COMMON_NAMES::WORLD_TF,
                                   rd_->getPelvisFrame());
    currentPelvisLocation.x += xOffset.at(m - 1);
    currentPelvisLocation.z += zOffset.at(m - 1);

    current_state_->transformPoint(currentPelvisLocation, currentWorldLocation, rd_->getPelvisFrame(),
                                   TOUGH_COMMON_NAMES::WORLD_TF);
    newFootStep->location = currentWorldLocation;
    newFootStep->orientation = current->orientation;
    newFootStep->robot_side = current->robot_side;
    newFootStep->trajectory_type = ihmc_msgs::FootstepDataRosMessage::OBSTACLE_CLEARANCE;

    currentWorldLocation = current->location;

    if (m > 2)
    {
      current_state_->transformPoint(currentWorldLocation, currentPelvisLocation, TOUGH_COMMON_NAMES::WORLD_TF,
                                     rd_->getPelvisFrame());
      currentPelvisLocation.x += xOffset.at(m - 2) - offset;
      currentPelvisLocation.z += zOffset.at(m - 1) + offset;
      current_state_->transformPoint(currentPelvisLocation, currentWorldLocation, rd_->getPelvisFrame(),
                                     TOUGH_COMMON_NAMES::WORLD_TF);

      newFootStep->position_waypoints.push_back(currentWorldLocation);
      newFootStep->trajectory_type = ihmc_msgs::FootstepDataRosMessage::CUSTOM;
    }

    newFootStep->position_waypoints.push_back(currentWorldLocation);
    list.footstep_data_list.push_back(*newFootStep);
  }

  this->walkGivenSteps(list);
  return true;
}

// wait till all the steps are taken
void RobotWalker::waitForSteps(const int numSteps)
{
  while (step_counter_ < numSteps && ros::ok())
  {
    ros::spinOnce();
    if ((ros::Time::now() - cbTime_) > ros::Duration(5))
    {
      // This is the case when the robot stopped walking due to external conditions(it fell down, there's an obstacle,
      // etc)
      break;
    }
    ros::Duration(0.1).sleep();
  }

  // reset back the step counter
  step_counter_ = 0;
  return;
}

void RobotWalker::alignFeet(const RobotSide side)
{
  geometry_msgs::Pose swingFootPose, finalPose;
  std::string swingFootFrame, stanceFootFrame;

  swingFootFrame = (side == RobotSide::LEFT) ? left_foot_frame_.data : right_foot_frame_.data;
  stanceFootFrame = (side == RobotSide::LEFT) ? right_foot_frame_.data : left_foot_frame_.data;
  current_state_->getCurrentPose(swingFootFrame, swingFootPose, stanceFootFrame);
  ROS_INFO_STREAM("swing foot pose " << swingFootPose);

  if (!areFeetAligned(swingFootPose))
  {
    finalPose.position.y += side == LEFT ? (FOOT_SEPARATION) : (-1 * FOOT_SEPARATION);
    finalPose.position.z -= rd_->getFootFrameOffset();
    finalPose.orientation.w = 1.0;
    current_state_->transformPose(finalPose, finalPose, stanceFootFrame, rd_->getWorldFrame());
    ROS_INFO_STREAM("final pose " << finalPose);
    stepAtPose(finalPose, side, true);
  }
}

bool RobotWalker::areFeetAligned(const geometry_msgs::Pose& footPose)
{
  // If the robot is not in double support, it does not make any sense in checking the alignment
  if (current_state_->isRobotInDoubleSupport())
  {
    return isFootPosAligned(footPose.position) && isFootRotAligned(footPose.orientation);
  }
  return false;
}

bool RobotWalker::isFootPosAligned(const geometry_msgs::Point& p1)
{
  return (fabs(p1.x) < FOOT_ALGMT_ERR_THRESHOLD && fabs(p1.y) < (FOOT_SEPARATION + FOOT_ALGMT_ERR_THRESHOLD));
}

bool RobotWalker::isFootRotAligned(const geometry_msgs::Quaternion& q1)
{
  return (fabs(q1.x) > FOOT_ROT_ERR_THRESHOLD || fabs(q1.y) > FOOT_ROT_ERR_THRESHOLD ||
          fabs(q1.z) > FOOT_ROT_ERR_THRESHOLD || fabs(q1.w) > FOOT_ROT_ERR_THRESHOLD);
}