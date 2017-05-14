#include <val_control/val_arm_navigation.h>
#include <ros/ros.h>
#include <val_moveit_planners/val_cartesian_planner.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <val_task1/val_task1_utils.h>
#include <val_control/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "test_cartesian_planner");
    ros::NodeHandle node_handle;

    // get the way points
    task1Utils task1_utils(node_handle);
    RobotStateInformer* robot_state = RobotStateInformer::getRobotStateInformer(node_handle);

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    geometry_msgs::Point centerWorld,centerPelvis;

    centerPelvis.x = 0.576;
    centerPelvis.y= -0.196;
    centerPelvis.z = -0.139;
    robot_state->transformPoint(centerPelvis,centerWorld,VAL_COMMON_NAMES::PELVIS_TF);


    geometry_msgs::Point startWorld,startPelvis;

    startPelvis.x = 0.432;
    startPelvis.y= -0.189;
    startPelvis.z = -0.138;
    robot_state->transformPoint(startPelvis,startWorld,VAL_COMMON_NAMES::PELVIS_TF);

    std::vector<geometry_msgs::Pose> points;
    std::vector<float> panelCoeffs {-0.002, 0.4591, 0.8884, -0.1502};

    //desired pose
    geometry_msgs::Pose grasp_pose;
    robot_state->getCurrentPose("/rightMiddleFingerPitch1Link",grasp_pose);

    task1_utils.getCircle3D(centerWorld, startWorld, grasp_pose, panelCoeffs, points, 0.125, 10);
    task1_utils.visulatise6DPoints(points);
    ROS_INFO("waypoints generated");

    ROS_INFO("Planning the trajectory");
    cartesianPlanner rightArmPlanner("rightArm", "/world");
    moveit_msgs::RobotTrajectory trajectory;
    rightArmPlanner.getTrajFromCartPoints(points, trajectory, false);


    moveit::planning_interface::MoveGroup::Plan my_plan;
    //    if (1)
    //    {
    //        ROS_INFO("Visualizing plan 1 (again)");
    //        display_trajectory.trajectory_start = my_plan.start_state_;
    //        display_trajectory.trajectory.push_back(trajectory);
    //        display_publisher.publish(display_trajectory);
    //        /* Sleep to give Rviz time to visualize the plan. */
    //        sleep(5.0);
    //        ros::spin();
    //    }




    //    ROS_INFO("executing on robot\n");
    //    armTrajectory armTraj(node_handle);
    //    armTraj.moveArmTrajectory(RIGHT, trajectory.joint_trajectory);
}
