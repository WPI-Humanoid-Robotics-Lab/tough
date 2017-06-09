#include <iostream>
#include<val_footstep/ValkyrieWalker.h>
#include<ihmc_msgs/FootstepDataListRosMessage.h>
#include"geometry_msgs/Pose2D.h"
#include "val_common/val_common_defines.h"

using namespace std;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_goal");
    ros::NodeHandle nh;
    ValkyrieWalker walk(nh, 1.0,1.0,0);
    geometry_msgs::Pose2D goal;
    if(argc == 4 )
    {
        goal.x = std::atof(argv[1]);
        goal.y = std::atof(argv[2]);
        goal.theta = std::atof(argv[3]);
        walk.walkToGoal(goal);
    }
    else cout<<"invalid input \n";

}

