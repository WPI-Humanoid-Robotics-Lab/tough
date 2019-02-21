#include <iostream>
#include <tough_footstep/robot_walker.h>
#include <ihmc_msgs/FootstepDataListRosMessage.h>
#include "geometry_msgs/Pose2D.h"

using namespace std;
int main(int argc, char** argv)
{
  if (argc == 2)
  {
    ros::init(argc, argv, "walk_forward");
    ros::NodeHandle nh;

    RobotWalker walk(nh, 1.0, 1.0, 0);
    std::vector<float> x_offset, y_offset;
    x_offset.push_back(std::atof(argv[1]));  // for right leg
    x_offset.push_back(std::atof(argv[1]));  // for left leg
    y_offset.push_back(0.0);
    y_offset.push_back(0.0);

    std::string msg = "Walking forward " + std::to_string(x_offset.front()) + " in a single step";
    ROS_INFO("%s", msg.c_str());
    walk.walkLocalPreComputedSteps(x_offset, y_offset,
                                   RIGHT);  // precomputed steps only execute the given values and does not try to get
                                            // both feet together at the end of the walk
  }
  else
  {
    std::string filename = string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_examples " << input_trace_filename << " <stride_length>";);
  }

  return 0;
}
