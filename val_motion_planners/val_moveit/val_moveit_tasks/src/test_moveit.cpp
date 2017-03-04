#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <val_control/val_arm_navigation.h>
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
    //    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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

    //    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    // set the target location
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.7;
    target_pose.position.z = 1.0;
    group.setPoseTarget(target_pose);


    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan((my_plan));

    if (success)
    {
        std::cout<<"sucessfully planned the trajectory"<<std::endl;
        std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator itp;

        std::cout << "joint names" << std::endl;
        for (it = my_plan.trajectory_.joint_trajectory.joint_names.begin(); it < my_plan.trajectory_.joint_trajectory.joint_names.end(); it++)
            std::cout << *it << std::endl;
        std::cout << "joint trajectories" << std::endl;
        for (itp = my_plan.trajectory_.joint_trajectory.points.begin(); itp < my_plan.trajectory_.joint_trajectory.points.end(); itp++)
            std::cout << *itp << std::endl;

        /* Sleep to give Rviz time to visualize the plan. */
        sleep(2.0);

        armTrajectory armTraj(node_handle);

        // fill the trajectory points
        //    ihmc_msgs::ArmTrajectoryRosMessage p;
        //    p.joint_trajectory_messages

        std::vector<armTrajectory::armJointData> trajPoints;
        trajPoints.clear();
        for (std::size_t i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); ++i)
        {
            trajectory_msgs::JointTrajectoryPoint pt = my_plan.trajectory_.joint_trajectory.points[i];
            armTrajectory::armJointData r;
            r.side = RIGHT;
            for (std::size_t j=0; j < pt.positions.size(); j++)
            {
                r.arm_pose.push_back(pt.positions[j]);
            }
            r.time = 0.1;
            trajPoints.push_back(r);
        }

        // execute the plan on robot
        ROS_INFO("executing on robot\n");
        armTraj.moveArmJoints(trajPoints);

    }
    else
    {
        ROS_INFO("planning failed");
    }
}
