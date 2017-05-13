#include <val_control/val_arm_navigation.h>
#include <ros/ros.h>
#include <val_moveit_planners/val_cartesian_planner.h>
#include <val_task1/val_task1_utils.h>
#include <val_control/robot_state.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "test_cartesian_planner");
    ros::NodeHandle node_handle;

    // get the way points
    task1Utils task1_utils(node_handle);
    RobotStateInformer* robot_state = RobotStateInformer::getRobotStateInformer(node_handle);

    geometry_msgs::Point center;
    center.x = 2.63;
    center.y = 0.956;
    center.z = 0.813;

    geometry_msgs::Point start;
    start.x = 2.62;
    start.y = 0.81;
    start.z = 0.823;

    std::vector<geometry_msgs::Pose> points;
    std::vector<float> panelCoeffs {0.0002, -0.459, 0.883, -0.2246};

    //desired pose
    geometry_msgs::Pose grasp_pose;
    robot_state->getCurrentPose("/rightMiddleFingerPitch1Link",grasp_pose);

    task1_utils.getCircle3D(center, start, grasp_pose, panelCoeffs, points, 0.125, 10);
    task1_utils.visulatise6DPoints(points);
    ROS_INFO("waypoints generated");

    ROS_INFO("Planning the trajectory");
    cartesianPlanner rightArmPlanner("rightArm", "/world");
    moveit_msgs::RobotTrajectory trajectory;
    rightArmPlanner.getTrajFromCartPoints(points, trajectory, false);

    ROS_INFO("executing on robot\n");
    armTrajectory armTraj(node_handle);
    armTraj.moveArmTrajectory(RIGHT, trajectory.joint_trajectory);
}
