
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tough_common/val_common_names.h>
#include <val_task1/move_handle.h>


int main(int argc, char** argv){

  ros::init(argc, argv, "panel_detector");
  ros::NodeHandle nh;

  move_handle handle(nh);
  ros::Rate loop_rate(10);

  geometry_msgs::Point center;
  center.x = 3.12;
  center.y = 0.923;
  center.z = 0.845;

  std::vector<geometry_msgs::Pose> points;
  std::vector<float> panelCoeffs {-0.3028,-0.3481,0.8872,0.5768};

  handle.createCircle(center, 0,panelCoeffs, points, M_PI/4);
  while(ros::ok())
  {
//    handle.follow_path();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
