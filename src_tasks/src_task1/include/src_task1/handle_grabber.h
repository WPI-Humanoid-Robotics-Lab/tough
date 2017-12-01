#ifndef HANDLE_GRABBER_H
#define HANDLE_GRABBER_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>
#include <tough_controller_interface/robot_state.h>
#include "tough_moveit_planners/tough_cartesian_planner.h"
#include "tough_controller_interface/wholebody_control_interface.h"

#define Y_OFFSET 0.05

class handle_grabber{
public:
    handle_grabber(ros::NodeHandle n);
    ~handle_grabber();
   void grasp_handles(const RobotSide side, const geometry_msgs::Point &goal, float executionTime=1.5f);
   void adjust_pose(const RobotSide side, const geometry_msgs::Point &goal, float executionTime=1.0f);

   geometry_msgs::QuaternionStamped leftHandOrientation() const;
   void setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation);

   geometry_msgs::QuaternionStamped rightHandOrientation() const;
   void setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation);

private:
   ros::NodeHandle nh_;
   tf::TransformListener listener_;
   ArmControlInterface armTraj_;
   GripperControlInterface gripper_;
   RobotStateInformer *current_state_;
   RobotDescription *rd_ ;
   geometry_msgs::QuaternionStamped leftHandOrientation_ ;
   geometry_msgs::QuaternionStamped rightHandOrientation_;
   CartesianPlanner* right_arm_planner_;
   CartesianPlanner* left_arm_planner_;
   WholebodyControlInterface* wholebody_controller_;
   /*Top Grip*/
   const std::vector<float> leftShoulderSeed_ = {-0.23, -0.72, 0.65, -1.51, 2.77, 0.0, 0.0};
   /*Side Grip*/
//   const std::vector<float> leftShoulderSeed_ = {-0.04, -0.24, 0.49, -1.30, 0.71, 0.61, -0.24};
   const std::vector<float> rightShoulderSeed_ = {-0.23, 0.72, 0.65, 1.51, 2.77, 0.0, 0.0};

};

#endif // HANDLE_GRABBER_H
