#include <navigation_common/fall_detector.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    const std::string TEXT_RED="\033[0;31m";
    const std::string TEXT_NC=  "\033[0m";

    ros::init(argc, argv, "robot_fall_detector");
    ros::NodeHandle nh;
    ros::Publisher logPub;
    logPub = nh.advertise<std_msgs::String>("/field/log", 10);
    FallDetector fall_detector(nh, "leftFoot", "pelvis", "world");


    ros::Rate rate(10.0);

    while (ros::ok())
    {
        if(fall_detector.isRobotFallen())
        {
            ROS_ERROR("!!!!!!!!!!!!!!!!!!!!....Robot has fallen....!!!!!!!!!!!!!!!!!!!!");
            std_msgs::String msg;
            msg.data = TEXT_RED+"!!!!!!!!!!!!!!!!!!!!....Robot has fallen....!!!!!!!!!!!!!!!!!!!!"+TEXT_NC;
            logPub.publish(msg);
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

