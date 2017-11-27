#ifndef SOLAR_PANEL_GRASP_H
#define SOLAR_PANEL_GRASP_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>
#include <tough_controller_interface/robot_state.h>
#include "tough_moveit_planners/tough_cartesian_planner.h"
#include "tough_controller_interface/wholebody_control_interface.h"

#define Y_OFFSET 0.05


class solar_panel_handle_grabber{
public:
    solar_panel_handle_grabber(ros::NodeHandle n);
    ~solar_panel_handle_grabber();
   bool grasp_handles(RobotSide side, const geometry_msgs::Pose &goal, bool isRotationRequired, float executionTime=2.0f);

   geometry_msgs::QuaternionStamped leftHandOrientation() const;
   void setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation);

   geometry_msgs::QuaternionStamped rightHandOrientation() const;
   void setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation);

private:
   ros::NodeHandle nh_;
   tf::TransformListener listener_;
   ArmControlInterface armTraj_;
   gripperControl gripper_;
   RobotStateInformer *current_state_;
   RobotDescription *rd_;
   CartesianPlanner* right_arm_planner_;
   CartesianPlanner* left_arm_planner_;
   wholebodyManipulation* wholebody_controller_;
   geometry_msgs::QuaternionStamped leftHandOrientationAngled_ ;
   geometry_msgs::QuaternionStamped rightHandOrientationAngled_;
   geometry_msgs::QuaternionStamped leftHandOrientationPerpen_ ;
   geometry_msgs::QuaternionStamped rightHandOrientationPerpen_;

   /*Top Grip*/
   const std::vector<float> leftShoulderSeed_ = {-0.23, -0.07, 0.75, -1.53, 1.21, -0.40, 0.0};
   /*Side Grip*/
//   const std::vector<float> leftShoulderSeed_ = {-0.04, -0.24, 0.49, -1.30, 0.71, 0.61, -0.24};
   const std::vector<float> rightShoulderSeed_ = {-0.81, -0.15, 1.65, 1.42, 1.28, 0.0, -0.23};

};

#endif // SOLAR_PANEL_GRASP_H
