#ifndef val_test_circle_h
#define val_test_circle_h

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>

#include <tough_common/val_common_names.h>
#include <tough_common/val_common_defines.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>
#include <val_gui/val_marker.h>



ros::Publisher* center_pub,*edge_pub,*execute_pub;
pthread_t input_thread;
std_msgs::Int16 execute_angle;
geometry_msgs::PoseStamped edge_pose, center_pose, center_start_pose, edge_start_pose;

bool edge_pose_received =false, center_pose_received =false;

void* input_thread_func(void *input_ptr);

void center_marker_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
void edge_marker_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

#endif
