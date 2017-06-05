#include <navigation_common/fall_detector.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_fall_detector");
    ros::NodeHandle nh;

    fallDetector fall_detector(nh, "leftFoot", "pelvis", "world");

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        if(fall_detector.isRobotFallen())
        {
            ROS_ERROR("!!!!!!!!!!!!!!!!!!!!....Robot Fallen....!!!!!!!!!!!!!!!!!!!!");
            int status = system("killall roscore rosmaster rosout gzserver gzclient roslaunch");
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

