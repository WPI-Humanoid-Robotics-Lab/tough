#include <iostream>
#include<val_footstep/ValkyrieWalker.h>
#include<ihmc_msgs/FootstepDataListRosMessage.h>
#include"geometry_msgs/Pose2D.h"
#include "val_common/val_common_defines.h"

using namespace std;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_forward");
    ros::NodeHandle nh;
    ValkyrieWalker walk(nh, 1.0,1.0,0);
    int numSteps;
    std::vector<float> x_offset,y_offset;
    if(argc == 3 )
    {
        numSteps = std::atoi(argv[1]);
        x_offset.push_back(std::atof(argv[2]));
        x_offset.push_back(std::atof(argv[2]));
        y_offset.push_back(0.0);
        y_offset.push_back(0.0);
        walk.walkLocalPreComputedSteps(x_offset,y_offset,RIGHT);
    }
    else cout<<"invalid input \n";

    return 0;

}

