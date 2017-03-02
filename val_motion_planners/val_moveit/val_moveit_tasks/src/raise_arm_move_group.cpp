/****
Reference :http://docs.ros.org/indigo/api/moveit_tutorials/html/
***/
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
void button_detected_callback(const std_msgs::Bool::ConstPtr& msg)
{
  
  
	//Creating a group for planning
        moveit::planning_interface::MoveGroup group("left_arm");
  	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	if(msg->data == true){
	ROS_INFO("Button detected!!");
	
          //Joint-space goal
	std::vector<double> group_joint_values;
      	group_joint_values.push_back(0.0);
      	group_joint_values.push_back(0.0);
      	group_joint_values.push_back(-1.59);
      	group_joint_values.push_back(-1.29);
      	group_joint_values.push_back(1.49);
      	group_joint_values.push_back(0.0);
      	group_joint_values.push_back(0.0);
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
	  
	  //Planning 
          bool success = group.plan(raise_arm_plan);
	  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
	  
          /* Sleep to give Rviz time to visualize the plan. */
	  sleep(5.0);
	  
          ROS_INFO("Raising Arm!");
	  
          // Executing the planned trajectory
	  group.move();
   }
   else{
	ROS_INFO("Button not detected!!");
	//Joint-space goal
	std::vector<double> group_joint_values;
      	group_joint_values.push_back(0.0);
      	group_joint_values.push_back(0.0);
      	group_joint_values.push_back(0.0);
      	group_joint_values.push_back(0.0);
      	group_joint_values.push_back(0.0);
      	group_joint_values.push_back(0.0);
      	group_joint_values.push_back(0.0);
	group.setJointValueTarget(group_joint_values);
	moveit::planning_interface::MoveGroup::Plan default_pose_plan;
	  
	  
	//Planning 
        bool success = group.plan(default_pose_plan);
	ROS_INFO("Visualizing plan (default pose) %s",success?"":"FAILED");    
	  
        /* Sleep to give Rviz time to visualize the plan. */
	sleep(5.0);
	  
        ROS_INFO("Going back to default pose");
	  
        // Executing the planned trajectory
	group.move();
	  
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

  //subsriber for detecting button signal
  ros::Subscriber sub = node_handle.subscribe("button_detected", 100, button_detected_callback);
  ros::spin();
}    
   



