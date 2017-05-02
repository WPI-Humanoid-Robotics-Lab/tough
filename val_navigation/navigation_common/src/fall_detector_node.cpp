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
            std::cout << "robot fallen"<< std::endl;
        }
        else
        {
            std::cout << "robot standing" <<std::endl;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

