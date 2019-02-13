#include <tough_controller_interface/arm_control_interface.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_arm_navigation");
  ros::NodeHandle nh;

  if (argc != 9)
  {
    std::string filename = std::string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_example " << input_trace_filename
                                                    << " <side_left=0,right=1> <q0> <q1> <q2> <q3> <q4> <q5> <q6>";);

    return -1;
  }

  ArmControlInterface armTraj(nh);
  std::vector<double> positions;
  for (int i = 0; i < 7; i++)
  {
    positions.push_back(std::stof(argv[i + 2]));
  }
  std::vector<std::vector<double> > armData;
  armData.push_back(positions);
  RobotSide side = argv[1][0] == '0' ? RobotSide::LEFT : RobotSide::RIGHT;

  armTraj.moveArmJoints(side, armData, 2.0f);
  ros::Duration(2).sleep();

  return 0;
}
