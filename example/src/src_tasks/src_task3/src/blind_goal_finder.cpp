#include <src_task3/val_task3_utils.h>
#include <ros/ros.h>


int main(int argc, char ** argv)
{
    ros::init (argc,argv,"blind_goal_finder");
    ros::NodeHandle nh;
    task3Utils task3(nh);
    geometry_msgs::Pose2D goal;
    task3.blindNavigation(goal);
    std::cout<<goal;
    return 0;
}

