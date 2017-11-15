#include <val_task3/climb_stairs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "climb_stairs_node");
    ros::NodeHandle nh_;

    climbStairs climbstairs(nh_);

    ROS_INFO("climbing stairs...");
    climbstairs.climb_stairs();

    return 0;
}
