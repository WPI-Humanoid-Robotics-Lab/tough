#include <val_controllers/val_chest_navigation.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_chest_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the chest");

  chestTrajectory chestTraj(nh);

  if(argc != 4)
    chestTraj.controlChest(10, 0, 0);
  else {
    float roll = std::atof(argv[1]);
    float pitch = std::atof(argv[2]);
    float yaw = std::atof(argv[3]);
    chestTraj.controlChest(roll, pitch, yaw);
  }

  while(ros::ok())
  {}

  return 0;
}
