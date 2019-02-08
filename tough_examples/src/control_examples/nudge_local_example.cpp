#include <tough_controller_interface/arm_control_interface.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_nudgeLocal");
  ros::NodeHandle nh;

  ArmControlInterface armTraj(nh);
  ros::Duration(0.5).sleep();

  if (argc == 3)
  {
    std::string side_str;
    RobotSide side;
    if (std::atoi(argv[1]) == 0)
    {
      side = LEFT;
      side_str = "left";
    }
    else
    {
      side = RIGHT;
      side_str = "right";
    }
    direction dir = (direction)std::atoi(argv[2]);
    armTraj.nudgeArm(side, dir);
    //    armTraj.nudgeArmLocal(side, dir);
  }
  else if (argc == 4)
  {
    std::string side_str;
    RobotSide side;
    if (std::atoi(argv[1]) == 0)
    {
      side = LEFT;
      side_str = "left";
    }
    else
    {
      side = RIGHT;
      side_str = "right";
    }
    direction dir = (direction)std::atoi(argv[2]);
    float step_size = std::atof(argv[3]);
    armTraj.nudgeArm(side, dir);
    //      armTraj.nudgeArmLocal(side, dir, step_size);
  }
  else
  {
    return 1;
  }

  ros::spinOnce();
  ros::Duration(2).sleep();

  return 0;
}
