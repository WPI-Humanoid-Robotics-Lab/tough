#include <tough_controller_interface/chest_control_interface.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <tough_common/tough_common_names.h>

int main(int argc, char** argv)
{
  const double TO_RADIANS = M_PI / 180.0;

  if (argc != 4)
  {
    std::string filename = std::string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_example " << input_trace_filename
                                                    << " <roll_in_degrees> <pitch_in_degrees> <yaw_in_degrees>";);
  }
  else
  {
    ros::init(argc, argv, "test_chest_navigation");
    ros::NodeHandle nh;
    ROS_INFO("Moving the chest");

    ChestControlInterface chestTraj(nh);
    float roll;
    float pitch;
    float yaw;
    roll = std::atof(argv[1]) * TO_RADIANS;
    pitch = std::atof(argv[2]) * TO_RADIANS;
    yaw = std::atof(argv[3]) * TO_RADIANS;

    ROS_INFO_STREAM("Moving chest angle to " + std::to_string(roll) + ", " + std::to_string(pitch) + ", " +
                    std::to_string(yaw));
    chestTraj.controlChest(roll, pitch, yaw);
    ros::spinOnce();
    ros::Duration(2).sleep();
    return 0;
  }
  return -1;
}
