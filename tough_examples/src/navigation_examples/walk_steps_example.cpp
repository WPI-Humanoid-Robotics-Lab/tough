#include <iostream>
#include <tough_footstep/robot_walker.h>
#include <ihmc_msgs/FootstepDataListRosMessage.h>
#include "geometry_msgs/Pose2D.h"

using namespace std;
int main(int argc, char** argv)
{
  if (argc == 3)
  {
    ros::init(argc, argv, "walk_steps");
    ros::NodeHandle nh;

    RobotWalker walk(nh, 1.0, 1.0, 0);
    RobotSide side;
    std::vector<float> x_offset, y_offset;
    walk.walkNStepsWRTPelvis(std::atof(argv[1]), std::atof(argv[2]));
    ROS_INFO_STREAM("Walking " << argv[1] << " steps with step length of " << argv[2]);
  }
  else
  {
    std::string filename = string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_examples " << input_trace_filename << " <num_of_steps> <step_length>");
  }
  return 0;
}
