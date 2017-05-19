#ifndef CABLEGRABBER_H
#define CABLEGRABBER_H


#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <val_control/val_arm_navigation.h>
#include <val_control/val_gripper_control.h>
#include <val_control/robot_state.h>
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_control/val_wholebody_manipulation.h"


class cableGrabber
{
public:
    cableGrabber(ros::NodeHandle n);
    ~cableGrabber();
    void grasp_cable(const armSide side, const geometry_msgs::Point &goal, float executionTime=2.0f);

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    armTrajectory armTraj_;
    gripperControl gripper_;
    RobotStateInformer *current_state_;
    geometry_msgs::QuaternionStamped leftHandOrientation_ ;
    geometry_msgs::QuaternionStamped rightHandOrientation_;
    cartesianPlanner* right_arm_planner_;
    cartesianPlanner* left_arm_planner_;
    wholebodyManipulation* wholebody_controller_;

    /*Top Grip*/
    const std::vector<float> leftShoulderSeed_ = {-0.23, -0.72, 0.65, -1.51, 2.77, 0.0, 0.0};
    const std::vector<float> rightShoulderSeed_ = {-0.23, 0.72, 0.65, 1.51, 2.77, 0.0, 0.0};
};

#endif // CABLEGRABBER_H




