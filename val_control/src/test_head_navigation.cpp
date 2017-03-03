#include <val_control/val_head_navigation.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_head_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the head");

  HeadTrajectory headTraj(nh);

  if(argc != 4)
    headTraj.controlHead(0, 0, 20, 2.0);
  else {
    float roll = std::atof(argv[1]);
    float pitch = std::atof(argv[2]);
    float yaw = std::atof(argv[3]);
    headTraj.controlHead(roll, pitch, yaw);
  }

  while(ros::ok())
  {}

  return 0;
}
