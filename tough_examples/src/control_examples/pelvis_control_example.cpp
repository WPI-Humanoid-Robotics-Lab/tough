#include <tough_controller_interface/pelvis_control_interface.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <tough_common/tough_common_names.h>

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::string filename = std::string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_examples " << input_trace_filename << " <desired_pelvis_height>";);
  }
  else
  {
    ros::init(argc, argv, "test_pelvis_navigation");
    ros::NodeHandle nh;
    ROS_INFO("Moving the pelvis");

    PelvisControlInterface pelvisTraj(nh);
    float height = std::atof(argv[1]);
    ROS_INFO_STREAM("moving pelvis to height " + std::to_string(height));
    pelvisTraj.controlPelvisHeight(height);
  }

  ros::spinOnce();
  ros::Duration(2).sleep();

  return 0;
}
