#include <val_task2/button_press.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "button_press_node");
  ros::NodeHandle nh;
  ROS_INFO("1");
  button_press bp(nh);
  ROS_INFO("2");
  armSide side = LEFT;
    ROS_INFO("3");
  geometry_msgs::Point goal;
    ROS_INFO("4");
  bp.grasp_button(side, goal);
  return 0;
}



