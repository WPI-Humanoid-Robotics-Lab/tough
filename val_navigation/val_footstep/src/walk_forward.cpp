#include <iostream>
#include<val_footstep/RobotWalker.h>
#include<ihmc_msgs/FootstepDataListRosMessage.h>
#include"geometry_msgs/Pose2D.h"
#include "val_common/val_common_defines.h"

using namespace std;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_forward");
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
    std::vector<float> x_offset,y_offset;
    if(argc == 2 )
    {
        x_offset.push_back(std::atof(argv[1]));
        x_offset.push_back(std::atof(argv[1]));
        y_offset.push_back(0.0);
        y_offset.push_back(0.0);

        log_msg("Walking forward " + std::to_string(x_offset.front()) + " in a single step");
        walk.walkLocalPreComputedSteps(x_offset,y_offset,RIGHT);
    }
    else {
        log_msg("Expected 1 argument, got " + std::to_string(argc - 1) + ". Exiting.");
    }

    log_msg("Walk complete");
    return 0;

}

