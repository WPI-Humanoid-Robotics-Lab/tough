#include <iostream>
#include<val_footstep/ValkyrieWalker.h>
#include<ihmc_msgs/FootstepDataListRosMessage.h>
#include"geometry_msgs/Pose2D.h"
int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_test");
    ros::NodeHandle nh;
    ValkyrieWalker walk(nh, 1.0,1.0,0);
    /*
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.transfer_time = 1;
    list.swing_time = 1;
    list.execution_mode =0;
    list.unique_id = -1;

    walk.WalkGivenSteps(list);
   

 //

 geometry_msgs::Pose2D goal;


 goal.x = 1.0;
 goal.y = 0.0;
 goal.theta = 0.0; 
 walk.WalkToGoal(goal);
*/

 //walk.WalkNStepsForward(4,0.6);
 //walk.WalkNStepsBackward(5,0.4);

    return 0;
}
