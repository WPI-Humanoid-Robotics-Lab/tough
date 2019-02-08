
#pragma once

// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>
#include <eigen_conversions/eigen_msg.h>

namespace dplanner
{
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);
void plantrajectory(ros::NodeHandle nh, std::vector<geometry_msgs::Pose>& pose, trajectory_msgs::JointTrajectory& traj);
void toROSJointTrajectory(const std::vector<descartes_core::TrajectoryPtPtr>& trajectory,
                          const descartes_core::RobotModel& model, const std::vector<std::string>& joint_names,
                          double time_delay, trajectory_msgs::JointTrajectory& traj);
}
