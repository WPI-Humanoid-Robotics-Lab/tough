#include <val_control/val_neck_navigation.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_neck_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the neck");

  NeckTrajectory neckTraj(nh);

  float tmp[] = { 1.57f, 1.57f };
  std::vector<float> v( tmp, tmp+2 );

  neckTraj.moveNeckJoints(v, 2.0f);

  while(ros::ok())
  {}

  return 0;
}
