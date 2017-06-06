#ifndef ROTATE_VALVE_H
#define ROTATE_VALVE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/val_gripper_control.h>
#include <val_controllers/robot_state.h>
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_controllers/val_wholebody_manipulation.h"
#include "val_footstep/ValkyrieWalker.h"

class rotateValve
{
public:
    rotateValve(ros::NodeHandle n);
    bool grab_valve(const geometry_msgs::Point &goal, float executionTime=2.0f);
    bool compute_traj(geometry_msgs::Point center, float radius, std::vector<geometry_msgs::Pose> &points);
    bool visualise_traj(std::vector<geometry_msgs::Pose> &points);
    bool move_valve(std::vector<geometry_msgs::Pose> points, float executionTime=2.0f);
    std::vector<geometry_msgs::Pose> poses;
    geometry_msgs::QuaternionStamped leftHandOrientationTop_,leftHandOrientationSide_,leftHandOrientationSideDown_,leftHandOrientationSideUp_;
    void reOrientbeforgrab(geometry_msgs::Point valveCenter);
    ~rotateValve();

private:
    ros::NodeHandle nh_;
    armTrajectory armTraj_;
    gripperControl gripper_;
    RobotStateInformer *current_state_;
    cartesianPlanner* left_arm_planner_;
    wholebodyManipulation* wholebody_controller_;
    chestTrajectory* chest_controller_;
    ValkyrieWalker walk_;
    const std::vector<float> LEFT_SHOULDER_SEED_INITIAL = {-0.81,0.0,0.65,-1.51,1.26,0.0,0.0};
    ros::Publisher marker_pub;
};

#endif // ROTATE_VALVE_H
