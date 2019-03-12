#include <footstep_planner/FootstepPlannerNode.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tough_footstep/robot_walker.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
#include "tough_common/robot_description.h"

RobotWalker* walk;
RobotStateInformer* current_state;
RobotDescription* rd_;
ihmc_msgs::FootstepDataListRosMessage list;
geometry_msgs::Pose pelvisPose;

void WalkToGoal(const geometry_msgs::Pose2D goal)
{
  current_state->getCurrentPose(rd_->getPelvisFrame(), pelvisPose);
  list.footstep_data_list.clear();
  list.default_transfer_duration = 1.0;
  list.default_swing_duration = 1.0;
  list.execution_mode = 0;
  list.unique_id = RobotWalker::id++;

  bool success = walk->getFootstep(goal, list);
  std::string result = success ? "Succeded" : "Failed";
}

double distanceBetweenPoints(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2)
{
  return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2));
}

void publish_footsteps_cb(const std_msgs::Empty msg)
{
  if (list.footstep_data_list.empty())
    return;

  geometry_msgs::Pose currPelvisPose;
  current_state->getCurrentPose(rd_->getPelvisFrame(), currPelvisPose);

  if (distanceBetweenPoints(currPelvisPose.position, pelvisPose.position) > 0.05)
    return;

  walk->walkGivenSteps(list, false);
  list.footstep_data_list.clear();
}

void nav_goal_cb(const geometry_msgs::PoseStamped::Ptr& goal_3d)
{
  geometry_msgs::Pose2D goal_2d;
  goal_2d.x = goal_3d->pose.position.x;
  goal_2d.y = goal_3d->pose.position.y;
  goal_2d.theta = tf::getYaw(goal_3d->pose.orientation);
  std::thread t1 = std::thread(WalkToGoal, goal_2d);
  t1.detach();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footstep_node");
  ros::NodeHandle nh;

  current_state = RobotStateInformer::getRobotStateInformer(nh);
  rd_ = RobotDescription::getRobotDescription(nh);
  walk = new RobotWalker(nh, 1.0f, 1.0f, 0, 0.1f);

  ros::Subscriber nav_goal_sub = nh.subscribe("/goal", 1, &nav_goal_cb);
  ros::Subscriber nav_goal_sub2 = nh.subscribe("/move_base_simple/goal", 1, &nav_goal_cb);
  ros::Subscriber publish_footsteps_sub = nh.subscribe("/approve_footsteps", 1, &publish_footsteps_cb);

  footstep_planner::FootstepPlannerNode planner;
  ros::spin();
  delete walk;
  return 0;
}
