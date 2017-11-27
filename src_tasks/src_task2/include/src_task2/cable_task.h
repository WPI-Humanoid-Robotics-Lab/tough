#ifndef CABLETASK_H
#define CABLETASK_H
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>
#include <tough_controller_interface/robot_state.h>
#include "tough_moveit_planners/tough_cartesian_planner.h"
#include "tough_controller_interface/wholebody_control_interface.h"
#include <stdlib.h>
#include <stdio.h>
#include "src_task_common/val_task_common_utils.h"
#include "tough_control_common/tough_control_common.h"
#include "src_task2/val_task2_utils.h"
#include "tough_control_common/tough_control_common.h"

#define Y_OFFSET 0.05


class CableTask{
public:
    CableTask(ros::NodeHandle n);
    ~CableTask();
   bool grasp_choke(RobotSide side, const geometry_msgs::Pose &goal, float executionTime=2.0f);
   bool grasp_cable(const geometry_msgs::Pose &goal, float executionTime=2.0f);
   bool insert_cable(const geometry_msgs::Point &goal, float executionTime=2.0f);
   bool rotate_cable1(const geometry_msgs::Pose &goal, float executionTime=2.0f);
   bool rotate_cable2(const geometry_msgs::Pose &goal, float executionTime=2.0f);
   bool rotate_cable3(const geometry_msgs::Pose &goal, float executionTime=2.0f);
   bool drop_cable(RobotSide side);
   bool allign_socket_axis(const geometry_msgs::Point &goal, float offset=0.1, float executionTime=2.0f);
private:
   ros::NodeHandle nh_;
   tf::TransformListener listener_;
   ArmControlInterface armTraj_;
   gripperControl gripper_;
   RobotStateInformer *current_state_;
   RobotDescription *rd_;
   CartesianPlanner* right_arm_planner_choke;
   CartesianPlanner* right_arm_planner_cable;
   CartesianPlanner* left_arm_planner_cable;
   CartesianPlanner* left_arm_planner_choke;
   chestTrajectory* chest_controller_;
   wholebodyManipulation* wholebody_controller_;
   geometry_msgs::QuaternionStamped rightHandOrientationTop_ ;
   geometry_msgs::QuaternionStamped rightHandOrientationAngle_ ;
   geometry_msgs::QuaternionStamped quat; // populate this value by experimentation
   task2Utils* task2_utils_;
   valControlCommon* control_common_;

   /*Top Grip*/
//   const std::vector<float> leftShoulderSeed_ = {-0.23, -0.07, 0.75, -1.53, 1.21, -0.40, 0.0};
   /*Side Grip*/
//   const std::vector<float> leftShoulderSeed_ = {-0.04, -0.24, 0.49, -1.30, 0.71, 0.61, -0.24};
//   const std::vector<float> rightShoulderSeed_ = {-0.81, -0.15, 1.65, 1.42, 1.28, 0.0, -0.23};

   const std::vector<float> leftShoulderSeedInitial_ = {-0.23,0.0,0.65,-1.49,1.29,0.0,0.0};
   const std::vector<float> rightShoulderSeedInitial_ = {-0.23,0.0,0.65,1.49,1.29,0.0,0.0};
   const std::vector<float> rightAfterGraspShoulderSeed_ = {-0.57, 1.09, 0.65, 1.1, 1.18, -0.19, 0.31};
   const std::vector<float> rightAfterGraspShoulderSeed2_ = {-0.57, 1.09, 0.65, 1.1, 0.30, -0.19, 0.31}; // more wrist yaw angle
   const std::vector<float> rightShoulderSeed_ = {-0.81,0.60,1.12,1.16,1.91,0.0,0.0};
   const std::vector<float> rightShoulderInHandSeed_ = {-0.81,1.11,0.65,1.14,-0.26,-0.19,0.30};
   const std::vector<float> rightAfterRotateSeed_ = {-1.35,0.76,1.70,1.07,1.44,0.62,0.06};
   const std::vector<float> rightCablePlaceSeed_ = {-1.56, 0.97, 2.18, 0.63, 0.86, 0.62, 0.15};
};

#endif // CABLETASK_H




