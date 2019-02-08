
#include <ros/ros.h>
#include <tough_common/val_common_defines.h>
#include <val_descartes_planner/val_descartes_palnner.h>
#include

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_dplanner");
  ros::NodeHandle nh;

  geometry_msgs::Point center;
  center.x = 2.628;
  center.y = 0.900;
  center.z = 0.835;

  geometry_msgs::Point start;
  start.x = 2.629;
  start.y = 0.900;
  start.z = 0.835;

  std::vector<geometry_msgs::Pose> points;

  // generate the way points

  ros::Duration(2).sleep();
  while (ros::ok())
  {
    trajectory_msgs::JointTrajectory traj;
    ROS_INFO("planning");
    ROS_INFO("Points size : %d", (int)points.size());
    dplanner::plantrajectory(nh, points, traj);

    // execute the trajectory
    // armTraj.moveArmTrajectory(armSide::RIGHT, traj);

    ros::spin();
  }
  return 0;
}
