#include <navigation_common/frame_tracking.h>

#define DEBOUNCE_COUNT 5 // debounces the motion to stationary transition (its a up dopwn debounce logic)

class walkTracking{
private:
    ros::NodeHandle nh_;
    // object for motion tracking
    frameTracking track_pelvis, track_leftFoot, track_rightFoot;


public:
    walkTracking(ros::NodeHandle nh): nh_(nh), track_pelvis(nh_, "pelvis", "world"),
        track_leftFoot(nh_, "rightFoot", "pelvis"),track_rightFoot(nh_, "leftFoot", "pelvis"){

    }

    bool isWalking(void);
};

bool walkTracking::isWalking(void)
{
    bool ret = false;
    static int debounce_count = 0;


    std::cout << debounce_count << std::endl;

    // if the robot is walking
    if(track_pelvis.isInMotion() == frame_track_status::FRAME_IN_MOTION ||
            track_leftFoot.isInMotion() == frame_track_status::FRAME_IN_MOTION ||
            track_rightFoot.isInMotion() == frame_track_status::FRAME_IN_MOTION)
    {
        ret = true;
        std::cout << "motion"<< std::endl;
    }
    // simple up down debounce for the transition from motion status to stationary status
    else if (debounce_count <= DEBOUNCE_COUNT)
    {
        // increment the debounce count
        debounce_count++;
        // consider robot still in motion
        ret = true;
        std::cout << "debounce motion"<< std::endl;
    }
    else
    {
        // reset the debounce count
        debounce_count = 0;
        ret = false;
        std::cout << "static" <<std::endl;
    }

    return ret;
}


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
            // std::cout << "motion"<< std::endl;
        }
        else
        {

            //std::cout << "static" <<std::endl;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
