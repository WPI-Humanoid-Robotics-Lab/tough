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
      int inputSide = argv[0];
      int   jointNumber= argv[1];
      float jointAngle = argv[2];

      armSide side;
      if(inputSide == 0){
          side = LEFT;
      } else {
          side = RIGHT;
      }
      armTraj.moveArmJoint(side, jointNumber,jointAngle);

    }
    ros::spinOnce();
    return 0;
}
