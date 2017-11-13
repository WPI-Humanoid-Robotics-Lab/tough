#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <ros/ros.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "test_moveit");
    ros::NodeHandle node_handle;

    /**************************************
     * ALWAYS START SPINNER IF USING MOVEIT
     *************************************/
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroup group("rightArm");

    ROS_INFO("set to state");
    // set the start state to the current state of the robot
    group.setStartStateToCurrentState();

    // get the cuurent joints and their positions
    std::vector<std::string> jNames;
    jNames = group.getActiveJoints();
    std::vector<double> jValues;
    jValues = group.getCurrentJointValues();
    std::vector<std::string>::iterator it;
    std::vector<double>::iterator itd;
    ROS_INFO("current state");
    for (it=jNames.begin(), itd=jValues.begin(); it<jNames.end(); it++, itd++){
        std::cout << *it << ": " << *itd << std::endl;
    }

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    // set the target location
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.38;
    target_pose.position.y = -0.7;
    target_pose.position.z = 1.0;
    group.setPoseTarget(target_pose);

    ROS_INFO("Planning now.");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan((my_plan));
    if (success)
    {
        ROS_INFO("sucessfully planned the trajectory");
        std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator itp;

        ROS_INFO("joint names");
        for (it = my_plan.trajectory_.joint_trajectory.joint_names.begin(); it < my_plan.trajectory_.joint_trajectory.joint_names.end(); it++)
            std::cout << *it << std::endl;
        std::cout << "joint trajectories" << std::endl;
        for (itp = my_plan.trajectory_.joint_trajectory.points.begin(); itp < my_plan.trajectory_.joint_trajectory.points.end(); itp++)
            std::cout << *itp << std::endl;

        /* Sleep to give Rviz time to visualize the plan. */
        sleep(2.0);

        ROS_INFO("executing on robot\n");
        armTrajectory armTraj(node_handle);
        armTraj.moveArmTrajectory(RIGHT, my_plan.trajectory_.joint_trajectory);

        ros::spin();
    }
    else
    {
        ROS_INFO("planning failed");
    }
}
