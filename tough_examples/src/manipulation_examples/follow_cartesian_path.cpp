#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <tough_common/tough_common_names.h>
#include <tough_common/robot_description.h>
#include <tough_moveit_planners/taskspace_planner.h>
#include <tough_controller_interface/wholebody_control_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_hand_in_traj");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  RobotDescription* rd = RobotDescription::getRobotDescription(nh);

  geometry_msgs::Pose pose_point;
  geometry_msgs::PoseArray pose_array;

  // Give a start point of the trajectory,
  pose_point.position.x = 0.4;
  pose_point.position.y = -0.7;
  pose_point.position.z = 0.3;

  pose_point.orientation.w = 1;
  pose_point.orientation.y = 0.0;
  pose_point.orientation.x = 0.0;
  pose_point.orientation.z = 0.0;

  pose_array.poses.push_back(pose_point);

  // Appending the subseqent points to be followed onto
  // existing trajectory and pushing into the vector.

  // follow a trajectory of vertically downwards by 0.2m
  for (int i = 0; i < 10; i++)
  {
    pose_point.position.z -= 0.02;
    pose_array.poses.push_back(pose_point);
  }

  std::string planning_groups = TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP;
  moveit_msgs::RobotTrajectory robot_traj;

  TaskspacePlanner taskspacePlanner(nh);
  taskspacePlanner.getTrajFromCartPoints(pose_array, planning_groups, robot_traj);

  WholebodyControlInterface wholeBodyCont(nh);
  wholeBodyCont.executeTrajectory(robot_traj);

  // Wait till the trajectory execution is complete.
  ros::Duration sleeptime = robot_traj.joint_trajectory.points.back().time_from_start;
  ros::Duration(sleeptime).sleep();

  return 0;
}
