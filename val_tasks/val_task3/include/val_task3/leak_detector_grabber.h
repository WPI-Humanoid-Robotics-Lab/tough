#ifndef LEAK_DETECTOR_GRAB_H
#define LEAK_DETECTOR_GRAB_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/val_gripper_control.h>
#include <val_controllers/robot_state.h>
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_controllers/val_wholebody_manipulation.h"
#include <val_task_common/val_task_common_utils.h>
#include <tf/transform_datatypes.h>

class leakDetectorGrabber{

public:
    leakDetectorGrabber(ros::NodeHandle nh);
    ~leakDetectorGrabber();

    void graspDetector(armSide side, geometry_msgs::Pose &goal, float executionTime=2.0f);


private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    armTrajectory armTraj_;
    gripperControl gripper_;
    RobotStateInformer *current_state_;
    cartesianPlanner *right_arm_planner_;
    cartesianPlanner *left_arm_planner_;
    wholebodyManipulation wholebody_controller_;
    geometry_msgs::QuaternionStamped leftHandOrientation_ ;
    geometry_msgs::QuaternionStamped rightHandOrientation_;


};

#endif // LEAK_DETECTOR_GRASP_H
