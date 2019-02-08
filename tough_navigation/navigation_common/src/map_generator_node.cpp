#include "navigation_common/map_generator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_generator");
  ros::NodeHandle n;
  ROS_INFO("Creating map generator object");
  MapGenerator mg(n);
  ROS_INFO("Created map generator object");
  ros::spin();
  return 0;
}
