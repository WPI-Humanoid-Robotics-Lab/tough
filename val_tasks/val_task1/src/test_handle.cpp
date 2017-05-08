
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
  center.x = 2.62;
  center.y = -1.21;
  center.z = 0.771;

  std::vector<geometry_msgs::Pose> points;
  std::vector<float> panelCoeffs {0.0075,0.4200,0.9075,-0.1875};

  while(ros::ok())
  {
    handle.createCircle(center, 0,panelCoeffs, points);
//    handle.follow_path();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
