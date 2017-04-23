#include <navigation_common/frame_tracking.h>

#define DEBOUNCE_COUNT 5 // debounces the motion to stationary transition (its a up dopwn debounce logic)

class walkTracking{
private:
    ros::NodeHandle nh_;
    // object for motion tracking
    frameTracking track_pelvis, track_leftFoot, track_rightFoot;


public:
    walkTracking(ros::NodeHandle nh): nh_(nh),
        track_pelvis(nh_, "pelvis", "world"),          // compare pelvis with world
        track_leftFoot(nh_, "rightFoot", "pelvis"),    // comapre right foot with pelvis
        track_rightFoot(nh_, "leftFoot", "pelvis")     // compare left foot with pelvis
    {

    }

    // determines if robot is waliking, returns True if walking
    bool isWalking(void);
};

bool walkTracking::isWalking(void)
{
    bool ret = false;
    static int debounce_count = 0;

    //std::cout << debounce_count << std::endl;

    // if the robot is walking
    if(track_pelvis.isInMotion() == frame_track_status::FRAME_IN_MOTION ||
        track_leftFoot.isInMotion() == frame_track_status::FRAME_IN_MOTION ||
        track_rightFoot.isInMotion() == frame_track_status::FRAME_IN_MOTION)
    {
        // robot is walking
        ret = true;

        // increment the debounce count
        debounce_count++;

        // wrap the debounce count
        debounce_count = ((debounce_count > DEBOUNCE_COUNT) ? DEBOUNCE_COUNT : debounce_count);

        //std::cout << "motion"<< std::endl;
    }
    // simple up down debounce for the transition from motion status to stationary status
    else if (debounce_count > 0)
    {
        // decrement the debounce count
        debounce_count--;

        // consider robot still in walking (in the debounce period)
        ret = true;

        //std::cout << "debounce motion"<< std::endl;
    }
    else
    {
        // reset the debounce count
        debounce_count = 0;

        // robot is not walking, its static
        ret = false;

        //std::cout << "static" <<std::endl;
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
