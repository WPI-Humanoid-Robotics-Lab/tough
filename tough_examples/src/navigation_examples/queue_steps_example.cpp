#include <tough_footstep/robot_walker.h>
#include <csignal>

void signalHandler(int signum)
{
  exit(signum);
}

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);
  ros::init(argc, argv, "walk_test", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  RobotWalker walk(nh, 0.8, 1.2);
  int side;
  while (ros::ok())
  {
    geometry_msgs::Pose goal;
    std::cout << "Enter <side | left = 0, right = 1> <x> <y> <z>\n";
    std::cin >> side >> goal.position.x >> goal.position.y >> goal.position.z;

    goal.orientation.w = 1;
    ROS_INFO_STREAM("Walking ");
    walk.stepAtPose(goal, (RobotSide)side, false, true);
  }

  return 0;
}
