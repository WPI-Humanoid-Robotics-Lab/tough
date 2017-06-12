#include <val_controllers/val_arm_navigation.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_nudgeOffset");
    ros::NodeHandle nh;
    ROS_INFO("Moving the arms");
    armTrajectory armTraj(nh);
    ros::Duration(1).sleep();
    armSide side;

    if(argc == 6){
        side = std::atoi(argv[1]) == 0 ? armSide::LEFT : armSide::RIGHT;
        if(std::atoi(argv[2]) ==0) armTraj.nudgeArmLocal(side, std::atof(argv[3]),std::atof(argv[4]),std::atof(argv[5]));
        else armTraj.nudgeArmPelvis(side, std::atof(argv[3]),std::atof(argv[4]),std::atof(argv[5]));
    }
    else
    {
        ROS_INFO(" 0 - local | 1 - pelvis, invalid input");
    }
    ros::spinOnce();
    ros::Duration(2).sleep();
    return 0;
}


