#include <src_task_common/val_walk_tracker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "walk_tracking");
    ros::NodeHandle nh;

    walkTracking track(nh);

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        if(track.isWalking())
        {
            std::cout << "walking"<< std::endl;
        }
        else
        {
            std::cout << "stationary" <<std::endl;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
