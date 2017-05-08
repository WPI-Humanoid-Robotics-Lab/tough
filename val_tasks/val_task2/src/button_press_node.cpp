#include <val_task2/button_press.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "button_press_node");
  ros::NodeHandle nh;
  button_press bp(nh);
  armSide side = LEFT;
  geometry_msgs::Point goal;
  bp.grasp_button(side, goal);
  return 0;
}



