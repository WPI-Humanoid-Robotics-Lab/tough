#include <iostream>
#include<val_footstep/ValkyrieWalker.h>
#include<ihmc_msgs/FootstepDataListRosMessage.h>
#include<mutex>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_test");
    ros::NodeHandle nh;
    ValkyrieWalker walk(nh, 0.75,0.75,2);
    /*
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.transfer_time = 1;
    list.swing_time = 1;
    list.execution_mode =0;
    list.unique_id = -1;

    walk.WalkGivenSteps(list);
   */

  walk.WalkNStepsForward(4,0.6);

 //walk.WalkNStepsBackward(4,0.4);

    return 0;
}
