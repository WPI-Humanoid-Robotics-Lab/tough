/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
void button_detected_callback(const std_msgs::Bool::ConstPtr& msg)
{
  
  if(msg->data == true){
	  ROS_INFO("Button detected!!");
	  //Planning 
  	  moveit::planning_interface::MoveGroup group("right_arm");
  	  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	  
          //Planning to a Joint-space goal
	  std::vector<double> group_joint_values;
	  group_joint_values[0]=0.0;
	  group_joint_values[1]=0.0;
	  group_joint_values[2]=0.0;
	  group_joint_values[3]=0.0;
	  group_joint_values[4]=1.266;
	  group_joint_values[5]=0.0;
	  group_joint_values[6]=0.0;
	  group.setJointValueTarget(group_joint_values);
	  
	  //Planning to a catersian-space goal
	  /*geometry_msgs::Pose target_pose;
  	  target_pose.orientation.w = 1.0;
	  target_pose.position.x = 0.033;
	  target_pose.position.y = 0.45;
	  target_pose.position.z = 0.97;
	  group.setPoseTarget(target_pose);*/

	  moveit::planning_interface::MoveGroup::Plan raise_arm_plan;
	  ROS_INFO("Planning to Raise Arm!!");
	  bool success = group.plan(raise_arm_plan);
	  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
	  /* Sleep to give Rviz time to visualize the plan. */
	sleep(5.0);
	  ROS_INFO("Raising Arm!");
	  // Execute the planned trajectory
	  group.move();
   }
   else{
	ROS_INFO("Button not detected!!");
   }

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "raise_arm_move_group");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(20.0); // This sleep is ONLY to allow Rviz to come up 
  
  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
moveit_msgs::DisplayTrajectory display_trajectory;
  //Create a subsriber for detecting button signal
  ros::Subscriber sub = node_handle.subscribe("button_detected", 100, button_detected_callback);
  ros::spin();
}    
   



