#ifndef MOVE_HANDLE_H
#define MOVE_HANDLE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <val_common/val_common_names.h>



class move_handle{

  ros::Publisher array_pub;
  ros::NodeHandle nh_;

public:
  move_handle (ros::NodeHandle);
  void createCircle(geometry_msgs::PoseStamped center, float,float,float,float );
  void visulatize(std::vector<geometry_msgs::PoseStamped>&);

};
#endif
