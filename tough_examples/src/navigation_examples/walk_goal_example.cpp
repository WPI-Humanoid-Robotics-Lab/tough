#include <iostream>
#include <tough_footstep/robot_walker.h>
#include <ihmc_msgs/FootstepDataListRosMessage.h>
#include "geometry_msgs/Pose2D.h"

using namespace std;
int main(int argc, char** argv)
{
  if (argc == 4)
  {
    ros::init(argc, argv, "walk_goal");
    ros::NodeHandle nh;

    RobotWalker walk(nh, 1.0, 1.0, 0);
    geometry_msgs::Pose2D goal;
    goal.x = std::atof(argv[1]);
    goal.y = std::atof(argv[2]);
    goal.theta = std::atof(argv[3]);
    std::string msg = "Walking to x: " + std::to_string(goal.x) + ", y: " + std::to_string(goal.y) +
                      ", theta: " + std::to_string(goal.theta);
    ROS_INFO("%s", msg.c_str());
    walk.walkToGoal(goal);
  }
  else
  {
    std::string filename = string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_examples " << input_trace_filename << " <x> <y> <theta_radians>");
  }

  return 0;
}
