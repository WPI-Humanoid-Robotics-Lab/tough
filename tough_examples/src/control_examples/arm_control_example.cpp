#include <tough_controller_interface/arm_control_interface.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  if (argc != 4)
  {
    std::string filename = std::string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_examples "
                        << input_trace_filename << " <side_left=0,right=1> <jointNumber> <desiredAngle_in_radians>";);
    return -1;
  }
  else
  {
    ros::init(argc, argv, "test_arm");
    ros::NodeHandle nh;
    ArmControlInterface armTraj(nh);

    ROS_INFO("Moving arms");
    int inputSide = std::atof(argv[1]);
    int jointNumber = std::atof(argv[2]);
    float jointAngle = std::atof(argv[3]);

    RobotSide side;
    std::string side_str;
    if (inputSide == 0)
    {
      side = LEFT;
    }
    else
    {
      side = RIGHT;
    }

    armTraj.moveArmJoint(side, jointNumber, jointAngle);
    ros::spinOnce();
    ros::Duration(2).sleep();
  }

  return 0;
}
