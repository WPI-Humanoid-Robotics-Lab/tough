#ifndef MOVE_HANDLE_H
#define MOVE_HANDLE_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <val_common/val_common_names.h>
#include <tough_controller_interface/robot_state.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <vector>
#include <stdio.h>      /* printf */
#include <iostream>
#include <tf/transform_listener.h>



class move_handle{

  ros::Publisher array_pub;
  ros::NodeHandle nh_;
  RobotStateInformer* robot_state_;
  RobotDescription* rd_;
  armTrajectory *armTraj;


public:
  move_handle (ros::NodeHandle);
  void follow_path(std::vector<geometry_msgs::Pose> &, armSide, geometry_msgs::Pose,std::vector<double>);
  void createCircle(geometry_msgs::Point center,int side, const std::vector<float> planeCoeffs, std::vector<geometry_msgs::Pose> &points,const float spin = 2*M_PI);
  void visulatize(std::vector<geometry_msgs::Pose> &);
private:
  std::vector<double> linspace(double , double , int );

};
#endif
