#include <val_controllers/val_arm_navigation.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_arm");
    ros::NodeHandle nh;
    ROS_INFO("Moving the arms");
    armTrajectory armTraj(nh);
    if(argc!=4)
    {
      ROS_ERROR("Invalid input!");
    }
    else{
      int inputSide = std::atof(argv[1]);
      int   jointNumber= std::atof(argv[2]);
      float jointAngle =std::atof(argv[3]);

      armSide side;
      if(inputSide == 0){
          side = LEFT;
      } else {
          side = RIGHT;
      }
      armTraj.moveArmJoint(side, jointNumber,jointAngle);

    }
    ros::spinOnce();
    ros::Duration(2).sleep();
    return 0;
}
