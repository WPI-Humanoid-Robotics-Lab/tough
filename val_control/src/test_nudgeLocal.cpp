#include <val_control/val_arm_navigation.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_nudgeLocal");
  ros::NodeHandle nh;
  ROS_INFO("Moving the arms");
  armTrajectory armTraj(nh);
  ros::Duration(1).sleep();

  if(argc == 3){
    armSide side;
    if(std::atoi(argv[1]) == 0){
      side = LEFT;
    } else {
      side = RIGHT;
    }
    direction dir = (direction)std::atoi(argv[2]);
    armTraj.nudgeArmLocal(side, dir);
  }
  else{
      ROS_INFO("Usage: %s <side> direction\n side is 0 or 1. read header file for direction indices", argv[0]);
  }
  ros::spin();
  return 0;
}

