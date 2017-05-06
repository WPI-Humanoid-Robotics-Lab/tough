#ifndef MOVE_HANDLE_H
#define MOVE_HANDLE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <val_common/val_common_names.h>
#include <val_control/robot_state.h>
#include <val_control/val_arm_navigation.h>

class move_handle{

  ros::Publisher array_pub;
  ros::NodeHandle nh_;
  RobotStateInformer* robot_state_;
  armTrajectory *right_armTraj,*left_armTraj;

public:
  move_handle (ros::NodeHandle);
  void createCircle(geometry_msgs::PoseStamped center, int, float,float,float,float );
  void visulatize(std::vector<geometry_msgs::PoseStamped>&);
  void follow_path(std::vector<geometry_msgs::PoseStamped>&,armSide,geometry_msgs::Pose);



};
#endif
