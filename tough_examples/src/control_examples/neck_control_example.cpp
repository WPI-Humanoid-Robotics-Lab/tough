#include <tough_controller_interface/head_control_interface.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <tough_common/tough_common_names.h>

int main(int argc, char** argv)
{
  if (argc != 4)
  {
    std::string filename = std::string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_example "
                        << input_trace_filename << " <side_left=0,right=1> <jointNumber> <desiredAngle_in_radians>";);
  }
  else
  {
    ros::init(argc, argv, "test_neck_navigation");
    ros::NodeHandle nh;
    float val1 = std::atof(argv[1]);
    float val2 = std::atof(argv[2]);
    float val3 = std::atof(argv[3]);

    ROS_INFO("Moving the neck");

    HeadControlInterface headTraj(nh);
    headTraj.moveNeckJoints({ { val1, val2, val3 } }, 2.0f);

    ros::spinOnce();
    ros::Duration(2).sleep();

    return 0;
  }
}
