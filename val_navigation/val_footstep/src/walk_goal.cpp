#include <iostream>
#include<val_footstep/RobotWalker.h>
#include<ihmc_msgs/FootstepDataListRosMessage.h>
#include"geometry_msgs/Pose2D.h"
#include "tough_common/val_common_defines.h"

using namespace std;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_goal");
    ros::NodeHandle nh;

    ros::Publisher log_pub = nh.advertise<std_msgs::String>(VAL_COMMON_NAMES::LOG_TOPIC, 10);
    const auto log_msg = [&log_pub](const std::string &str) {
        std_msgs::String msg;
        msg.data = ros::this_node::getName() + ": " + str;
        log_pub.publish(msg);
        ROS_INFO("%s", msg.data.c_str());
    };

    // wait a reasonable amount of time for the subscriber to connect
    ros::Time wait_until = ros::Time::now() + ros::Duration(0.5);
    while (log_pub.getNumSubscribers() == 0 && ros::Time::now() < wait_until) {
        ros::spinOnce();
        ros::WallDuration(0.1).sleep();
    }


    RobotWalker walk(nh, 1.0,1.0,0);
    geometry_msgs::Pose2D goal;
    if(argc == 4 )
    {
        goal.x = std::atof(argv[1]);
        goal.y = std::atof(argv[2]);
        goal.theta = std::atof(argv[3]);
        log_msg("Walking to x: " + std::to_string(goal.x) + ", y: " + std::to_string(goal.y) +
                        ", theta: " + std::to_string(goal.theta));
        walk.walkToGoal(goal);
    } else {
        log_msg("Expected 3 arguments, but got " + std::to_string(argc - 1) + ". Exiting.");
    }

    log_msg("Walking complete");
    return 0;
}

