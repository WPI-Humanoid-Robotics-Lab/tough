#include "tough_footstep/RobotWalker.h"

int main(int argc, char **argv)
{
    //initializing ros node
    ros::init(argc, argv, "walker_node");
    ros::NodeHandle nh;

    //This should make the robot actually walk three steps - wrt to the world frame     
    RobotWalker r(nh, 2.0, 2.0, 1, 0.25);
    r.walkNSteps(3, 0.1, 0.1, true, LEFT, true);

    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}
