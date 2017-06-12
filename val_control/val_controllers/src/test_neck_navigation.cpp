#include <val_controllers/val_head_navigation.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_neck_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the neck");

  HeadTrajectory headTraj(nh);

  headTraj.moveNeckJoints({{ 0.0f, -1.0f, 0.0f }}, 2.0f);

  ros::spinOnce();

  return 0;
}
