#include <val_task_common/val_walk_tracker.h>

walkTracking::walkTracking(ros::NodeHandle nh): nh_(nh),
    track_pelvis_(nh_, "pelvis", "world"),          // compare pelvis with world
    track_leftFoot_(nh_, "rightFoot", "pelvis"),    // comapre right foot with pelvis
    track_rightFoot_(nh_, "leftFoot", "pelvis")     // compare left foot with pelvis
{
}

walkTracking::~walkTracking()
{

}

bool walkTracking::isWalking(void)
{
    bool ret = false;
    static int debounce_count = 0;

    // std::cout << debounce_count << std::endl;

    // if the robot is walking
    if(track_pelvis_.isInMotion() == frame_track_status::FRAME_IN_MOTION ||
       track_leftFoot_.isInMotion() == frame_track_status::FRAME_IN_MOTION ||
       track_rightFoot_.isInMotion() == frame_track_status::FRAME_IN_MOTION)
    {
        // robot is walking
        ret = true;

        // set the debounce count
        debounce_count = DEBOUNCE_COUNT;

        // std::cout << "motion"<< std::endl;
    }
    // simple up down debounce for the transition from motion status to stationary status
    else if (debounce_count > 0)
    {
        // decrement the debounce count
        debounce_count--;

        // consider robot still in walking (in the debounce period)
        ret = true;

        // std::cout << "debounce motion"<< std::endl;
    }
    else
    {
        // reset the debounce count
        debounce_count = 0;

        // robot is not walking, its static
        ret = false;

       // std::cout << "static" <<std::endl;
    }

    return ret;
}
