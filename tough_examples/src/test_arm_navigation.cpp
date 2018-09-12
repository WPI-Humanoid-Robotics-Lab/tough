#include <tough_controller_interface/arm_control_interface.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_arm_navigation");
    ros::NodeHandle nh;

    ros::Publisher log_pub = nh.advertise<std_msgs::String>(TOUGH_COMMON_NAMES::LOG_TOPIC, 10);
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

    if (argc != 9) {
        log_msg("Expected 8 arguments, got " + std::to_string(argc - 1) + ". Exiting.");
        return -1;
    }

    ArmControlInterface armTraj(nh);
    std::vector<float> positions;
    for (int i = 0; i < 7; i ++){
        positions.push_back(std::stof(argv[i+2]));
    }
    std::vector< std::vector<float> > armData;
    armData.push_back(positions);
    RobotSide side = argv[1][0] == '0' ? RobotSide::LEFT : RobotSide::RIGHT;
    std::string side_str = (side == RobotSide::LEFT) ? "left" : "right";

    log_msg("Moving " + side_str + " arm to given joint angles");
//    armData.push_back(positions);
    armTraj.moveArmJoints(side, armData, 2.0f);

    ros::Duration(1).sleep();

    log_msg("Motion complete");
    return 0;
}
