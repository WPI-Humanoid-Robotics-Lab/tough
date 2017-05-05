#include <val_task2/button_press.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "button_press_node");
  ros::NodeHandle nh;
  geometry_msgs::Pose point;
  point.position.x = 0.5;
  point.position.y = 0.5;
  point.position.z = 0.5;
  point.orientation.x = 0;
  point.orientation.y = 0;
  point.orientation.z = 0;
  point.orientation.w = 1;
  button_press bp(nh);
  bp.pressButton(point);
  while(ros::ok())
  {}
  return 0;
}
