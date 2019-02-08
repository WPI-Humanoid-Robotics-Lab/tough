#include <ros/ros.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_moveit_planners/tough_cartesian_planner.h>
#include <tough_common/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "tough_controller_interface/wholebody_control_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_palnner_example");
  ros::NodeHandle node_handle;

  RobotStateInformer* robot_state = RobotStateInformer::getRobotStateInformer(node_handle);
  RobotDescription* rd_ = RobotDescription::getRobotDescription(node_handle);

  geometry_msgs::Pose pose1, pose2;

  pose1.position.x = 3.0;
  pose1.position.y = 0.945;
  pose1.position.z = 0.845;
  pose1.orientation.w = 1;

  pose2.position.x = 2.6;
  pose2.position.y = 0.84;
  pose2.position.z = 0.82;
  pose2.orientation.w = 1;

  geometry_msgs::Pose grasp_pose;
  robot_state->getCurrentPose(rd_->getRightEEFrame(), grasp_pose);

  std::vector<geometry_msgs::Pose> poses;
  poses.push_back(grasp_pose);
  poses.push_back(pose1);
  poses.push_back(pose2);

  ROS_INFO("Planning the trajectory");
  CartesianPlanner rightArmPlanner(TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP, TOUGH_COMMON_NAMES::WORLD_TF);
  moveit_msgs::RobotTrajectory trajectory;
  rightArmPlanner.getTrajFromCartPoints(poses, trajectory, false);

  ROS_INFO("executing on robot\n");

  WholebodyControlInterface msg(node_handle);
  msg.executeTrajectory(trajectory.joint_trajectory);

  ros::Duration(5).sleep();
}
