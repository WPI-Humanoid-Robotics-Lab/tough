
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

  geometry_msgs::Point center;
  center.x = 3.10;
  center.y = .9;
  center.z = 0.84;

  std::vector<geometry_msgs::Pose> points;
  std::vector<float> panelCoeffs {-0.3028,-0.3481,0.8872,0.5768};

  while(ros::ok())
  {
    handle.createCircle(center, 0,panelCoeffs, points);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
