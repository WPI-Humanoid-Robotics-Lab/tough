
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <val_common/val_common_names.h>
#include <val_task1/move_handle.h>


int main(int argc, char** argv){

  ros::init(argc, argv, "panel_detector");
  ros::NodeHandle nh;

  move_handle handle(nh);
  ros::Rate loop_rate(10);

  geometry_msgs::PoseStamped center;
  center.pose.position.x = 3.10;
  center.pose.position.y = .9;
  center.pose.position.z = 0.84;
  center.pose.orientation.x  = 0;
  center.pose.orientation.y  = 0;
  center.pose.orientation.z  = 0;
  center.pose.orientation.w  = 1;

  std::vector<geometry_msgs::PoseStamped> points;


  while(ros::ok())
  {
    handle.createCircle(center, 0, -0.3028,-0.3481,0.8872,0.5768 );
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
