#ifndef CABLETASK_H
#define CABLETASK_H


#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/val_gripper_control.h>
#include <val_controllers/robot_state.h>
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_controllers/val_wholebody_manipulation.h"
#include "val_controllers/val_chest_navigation.h"

class cableTask
{
public:
    cableTask(ros::NodeHandle n);
    ~cableTask();
    bool grasp_cable(const armSide side, const geometry_msgs::Point &goal, float executionTime=2.0f);
    bool insert_cable(const armSide side, const geometry_msgs::Point &goal, float executionTime=2.0f);

private:
    ros::NodeHandle nh_;
    armTrajectory armTraj_;
    gripperControl gripper_;
    RobotStateInformer *current_state_;
    geometry_msgs::QuaternionStamped leftHandOrientationTop_ ;
    geometry_msgs::QuaternionStamped rightHandOrientationTop_;
    geometry_msgs::QuaternionStamped leftHandOrientationSide_ ;
    geometry_msgs::QuaternionStamped rightHandOrientationSide_;

    cartesianPlanner* right_arm_planner_;
    cartesianPlanner* left_arm_planner_;
    wholebodyManipulation* wholebody_controller_;
    chestTrajectory * chest_controller_;

    /*Top Grip*/
    const std::vector<float> leftShoulderSeed_ = {-0.23, -0.72, 0.65, -1.51, 2.77, 0.0, 0.0};
    const std::vector<float> rightShoulderSeed_ = {-0.57, 1.09, 0.65, 1.14, 2.78, -0.19, 0.31}; //approach 1
    const std::vector<float> rightAfterGraspShoulderSeed_ = {-0.57, 1.09, 0.65, 1.1, 1.18, -0.19, 0.31};

};

#endif // CABLETASK_H




