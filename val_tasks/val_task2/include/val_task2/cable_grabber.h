#ifndef CABLEGRABBER_H
#define CABLEGRABBER_H


#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/val_gripper_control.h>
#include <val_controllers/robot_state.h>
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_controllers/val_wholebody_manipulation.h"
#include "val_controllers/val_chest_navigation.h"

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
    chestTrajectory * chest_controller_;

    /*Top Grip*/
    const std::vector<float> leftShoulderSeed_ = {-0.23, -0.72, 0.65, -1.51, 2.77, 0.0, 0.0};
    //    const std::vector<float> rightShoulderSeed_ = {-0.23, 0.72, 0.65, 1.51, 2.77, 0.0, 0.0}; //approach default
    const std::vector<float> rightShoulderSeed_ = {-0.57, 1.09, 0.65, 1.14, 2.78, -0.19, 0.31}; //approach 1
    //        const std::vector<float> rightShoulderSeed_ = {-0.18,0.50,0.50,1.50,2.11,0.0,0.0}; //approach 2
    //    const std::vector<float> rightShoulderSeed_ = {-1,1.16,1.60,0.78,1.64,0.0,0.0}; //approach 3
     const std::vector<float> rightAfterGraspShoulderSeed_ = {-0.57, 1.09, 0.65, 1.1, 1.18, -0.19, 0.31};
    ros::Publisher marker_pub_;

};

#endif // CABLEGRABBER_H




