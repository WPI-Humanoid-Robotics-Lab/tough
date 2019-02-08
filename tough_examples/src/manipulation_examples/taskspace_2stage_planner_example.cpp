#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

#include "tough_common/tough_common_names.h"
#include "tough_moveit_planners/taskspace_planner.h"
#include "tough_controller_interface/wholebody_control_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planning_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  WholebodyControlInterface wb_controller(nh);
  geometry_msgs::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  pose.header.frame_id = "pelvis";

  if (argc != 4)
  {
    pose.pose.position.x = 0.4;
    pose.pose.position.y = 0.8;
    pose.pose.position.z = 0.2;
  }
  else
  {
    pose.pose.position.x = std::atof(argv[1]);
    pose.pose.position.y = std::atof(argv[2]);
    pose.pose.position.z = std::atof(argv[3]);
  }
  TaskspacePlanner man(nh);
  moveit_msgs::RobotTrajectory trajectory_msg;
  std::string planner_group1 = TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP;
  std::string planner_group2 = TOUGH_COMMON_NAMES::RIGHT_ARM_10DOF_GROUP;

  // plan and execute 7 DOF trajectory with higher goal tolerance
  man.setAngleTolerance(0.2);
  man.setPositionTolerance(0.2);
  man.getTrajectory(pose, planner_group1, trajectory_msg);
  wb_controller.executeTrajectory(trajectory_msg);

  // wait for the trajectory to execute
  ros::Duration sleeptime = trajectory_msg.joint_trajectory.points.back().time_from_start;
  sleeptime.sleep();

  // plan and execute 10DOF trajectory with lower goal tolerance
  man.setAngleTolerance(0.01);
  man.setPositionTolerance(0.01);
  man.getTrajectory(pose, planner_group1, trajectory_msg);
  wb_controller.executeTrajectory(trajectory_msg);

  return 0;
}
