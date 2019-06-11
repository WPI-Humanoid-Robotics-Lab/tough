#include <tough_controller_interface/head_control_interface.h>
#include <stdlib.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <tough_common/tough_common_names.h>

int main(int argc, char** argv)
{

  if (argc != 4)
  {
    std::string filename = std::string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_examples " << input_trace_filename
                                                     << " <roll in degrees> <pitch in degrees> <yaw in degrees>");
    return -1;
  }
  else
  {
    const float degToRad = M_PI / 180.0f;
    ros::init(argc, argv, "test_head_navigation", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    HeadControlInterface headTraj(nh);
    ROS_INFO("Moving the head");
    float roll = std::atof(argv[1]) * degToRad;
    float pitch = std::atof(argv[2]) * degToRad;
    float yaw = std::atof(argv[3]) * degToRad;

    headTraj.moveHead(roll, pitch, yaw);
    ros::Duration(2).sleep();
    return 0;
  }
}
