#include <iostream>
#include<val_footstep/ValkyrieWalker.h>
#include<ihmc_msgs/FootstepDataListRosMessage.h>
#include"geometry_msgs/Pose2D.h"
#include "val_common/val_common_defines.h"
#include "val_controllers/robot_state.h"


using namespace std;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_rotate");
    ros::NodeHandle nh;
    ValkyrieWalker walk(nh, 1.0,1.0,0);
    geometry_msgs::Pose2D goal;

    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    walk.getCurrentStep(RIGHT, *current);

    // get current position
    goal.x = current->location.x;
    goal.y = current->location.y;
    if(argc == 2)
    {
        goal.theta = tf::getYaw(current->orientation)+std::atof(argv[1]);
        walk.walkToGoal(goal);
    }
    else cout<<"invalid input \n";

}


