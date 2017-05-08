#ifndef MOVE_HANDLE_H
#define MOVE_HANDLE_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <val_common/val_common_names.h>
#include <val_control/robot_state.h>
#include <val_control/val_arm_navigation.h>
#include <vector>
#include <stdio.h>      /* printf */
#include <iostream>


class move_handle{

  ros::Publisher array_pub;
  ros::NodeHandle nh_;
  RobotStateInformer* robot_state_;
  armTrajectory *right_armTraj,*left_armTraj;

public:
  move_handle (ros::NodeHandle);
  void follow_path(std::vector<geometry_msgs::Pose> &, armSide, geometry_msgs::Pose);
  void createCircle(geometry_msgs::Point center,int side, const std::vector<float> planeCoeffs, std::vector<geometry_msgs::Pose> &points);
  void visulatize(std::vector<geometry_msgs::Pose> &);
  std::vector<double> linspace(double , double , int );

};
#endif
