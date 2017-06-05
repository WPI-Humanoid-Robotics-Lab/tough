#ifndef HANDLE_GRABBER_H
#define HANDLE_GRABBER_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/val_gripper_control.h>
#include <val_controllers/robot_state.h>
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_controllers/val_wholebody_manipulation.h"

#define Y_OFFSET 0.05

class handle_grabber{
public:
    handle_grabber(ros::NodeHandle n);
    ~handle_grabber();
   void grasp_handles(const armSide side, const geometry_msgs::Point &goal, float executionTime=1.5f);
   void adjust_pose(const armSide side, const geometry_msgs::Point &goal, float executionTime=1.0f);

   geometry_msgs::QuaternionStamped leftHandOrientation() const;
   void setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation);

   geometry_msgs::QuaternionStamped rightHandOrientation() const;
   void setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation);

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
   /*Side Grip*/
//   const std::vector<float> leftShoulderSeed_ = {-0.04, -0.24, 0.49, -1.30, 0.71, 0.61, -0.24};
   const std::vector<float> rightShoulderSeed_ = {-0.23, 0.72, 0.65, 1.51, 2.77, 0.0, 0.0};

};

#endif // HANDLE_GRABBER_H
