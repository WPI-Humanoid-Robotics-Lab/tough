#include <iostream>
#include <tough_footstep/robot_walker.h>
#include <ihmc_msgs/FootstepDataListRosMessage.h>
#include "geometry_msgs/Pose2D.h"
#include "tough_common/robot_state.h"

using namespace std;
int main(int argc, char** argv)
{
  if (argc == 2)
  {
    ros::init(argc, argv, "walk_rotate");
    ros::NodeHandle nh;

    RobotWalker walk(nh, 1.0, 1.0, 0);
    walk.walkRotate(std::atof(argv[1]));
  }
  else
  {
    std::string filename = string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_example " << input_trace_filename << " <theta_radians>");
  }

  return 0;
}
