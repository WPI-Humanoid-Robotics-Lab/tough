#ifndef val_circle_h
#define val_circle_h 1
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <interactive_markers/interactive_marker_server.h>
#include <val_control/val_arm_navigation.h>
#include <val_common/val_common_names.h>
#include <val_common/val_common_defines.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <val_gui/val_marker.h>
#include <visualization_msgs/MarkerArray.h>


//vars
std::vector<geometry_msgs::PoseStamped*> *right_poses,*left_poses;
std::vector<armTrajectory::armTaskSpaceData> *arm_data;
geometry_msgs::PoseStamped center_pose, edge_pose;
int execute_angle;
ros::Publisher array_pub;
tf::TransformListener* listener;
armTrajectory *right_armTraj,*left_armTraj;
bool left_pose_received=false,right_pose_received=false,center_pose_received,edge_pose_received;


void execute_callback(const std_msgs::Int16& msg);
void pose_callback(const geometry_msgs::PoseStamped& msg);
void center_callback(const geometry_msgs::PoseStamped& msg);
void edge_callback(const geometry_msgs::PoseStamped& msg);
vector<geometry_msgs::PoseStamped*>* generate_circle_poses(geometry_msgs::PoseStamped& center, int angle);
std::vector<armTrajectory::armTaskSpaceData>* generate_task_space_data(std::vector<geometry_msgs::PoseStamped*>* input_poses, armSide input_side, float desired_time=6.0f);
geometry_msgs::PoseStamped transform_pose_simple(const geometry_msgs::PoseStamped *from_pose,std::string to_frame);
void marker_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );



inline double deg2rad (double degrees) {
    static const double pi_on_180 = 4.0 * atan (1.0) / 180.0;
    return degrees * pi_on_180;
}

#endif
