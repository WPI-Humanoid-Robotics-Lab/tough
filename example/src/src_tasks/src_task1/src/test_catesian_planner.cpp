#include <tough_controller_interface/arm_control_interface.h>
#include <ros/ros.h>
#include <tough_moveit_planners/tough_cartesian_planner.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <src_task1/val_task1_utils.h>
#include <tough_controller_interface/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "tough_controller_interface/wholebody_control_interface.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "test_cartesian_planner");
    ros::NodeHandle node_handle;

    // get the way points
    task1Utils task1_utils(node_handle);
    RobotStateInformer* robot_state = RobotStateInformer::getRobotStateInformer(node_handle);
    RobotDescription* rd_ = RobotDescription::getRobotDescription(node_handle);
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    geometry_msgs::Point centerWorld,centerPelvis;

    centerPelvis.x = 2.66;
    centerPelvis.y= 0.945;
    centerPelvis.z = 0.845;
//    robot_state->transformPoint(centerPelvis,centerWorld,VAL_COMMON_NAMES::PELVIS_TF);


    geometry_msgs::Point startWorld,startPelvis;

    startPelvis.x = 2.6;
    startPelvis.y= 0.84;
    startPelvis.z = 0.82;
//    robot_state->transformPoint(startPelvis,startWorld,VAL_COMMON_NAMES::PELVIS_TF);

    std::vector<geometry_msgs::Pose> points;
//    std::vector<float> panelCoeffs {-0.002, 0.4591, 0.8884, -0.1502};
  std::vector<float> panelCoeffs {-0.0053, -0.4647, 0.8840, -0.1993};
    //desired pose
    geometry_msgs::Pose grasp_pose;
    robot_state->getCurrentPose(rd_->getRightEEFrame(),grasp_pose);
    task1_utils.getCircle3D(centerPelvis, startPelvis, grasp_pose.orientation, panelCoeffs, points, handleDirection::ANTICLOCK_WISE, 0.125, 10);
//    task1_utils.getCircle3D(centerWorld, startWorld, grasp_pose, panelCoeffs, points, 0.125, 10);
    task1_utils.visulatise6DPoints(points);
    ROS_INFO("waypoints generated");

    ROS_INFO("Planning the trajectory");
    CartesianPlanner rightArmPlanner("rightPalm", VAL_COMMON_NAMES::WORLD_TF);
    moveit_msgs::RobotTrajectory trajectory;
    rightArmPlanner.getTrajFromCartPoints(points, trajectory, false);




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




    ROS_INFO("executing on robot\n");
//            ArmControlInterface armTraj(node_handle);
//            armTraj.moveArmTrajectory(RIGHT, trajectory.joint_trajectory);

    WholebodyControlInterface msg(node_handle);
    msg.executeTrajectory(RIGHT,trajectory.joint_trajectory);

}
