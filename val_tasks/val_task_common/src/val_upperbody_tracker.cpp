# include <val_task_common/val_upperbody_tracker.h>

uuperBodyTracker::uuperBodyTracker(ros::NodeHandle nh): nh_(nh),
    // torso
    track_torso_yaw_(nh_, "torsoYawLink", "pelvis"),
    track_torso_pitch_(nh_, "torsoPitchLink", "pelvis"),
    track_torso_roll_ (nh_, "torso", "pelvis"),
    // left arm
    track_lshoulder_pitch_(nh_, "leftShoulderPitchLink", "pelvis"),
    track_lshoulder_roll_(nh_, "leftShoulderRollLink", "pelvis"),
    track_lshoulder_yaw_(nh_, "leftShoulderYawLink", "pelvis"),
    track_lelbow_pitch_(nh_, "leftElbowPitchLink", "pelvis"),
    track_lforearm_yaw_(nh_, "leftForearmLink", "pelvis"),
    track_lwrist_roll_(nh_, "leftWristRollLink", "pelvis"),
    track_lwrist_pitch_(nh_, "leftPalm", "pelvis"),
    // right arm
    track_rshoulder_pitch_(nh_, "rightShoulderPitchLink", "pelvis"),
    track_rshoulder_roll_(nh_, "rightShoulderRollLink", "pelvis"),
    track_rshoulder_yaw_(nh_, "rightShoulderYawLink", "pelvis"),
    track_relbow_pitch_(nh_, "rightElbowPitchLink", "pelvis"),
    track_rforearm_yaw_(nh_, "rightForearmLink", "pelvis"),
    track_rwrist_roll_(nh_, "rightWristRollLink", "pelvis"),
    track_rwrist_pitch_(nh_, "rightPalm", "pelvis")
{

}

uuperBodyTracker::~uuperBodyTracker()
{

}

bool uuperBodyTracker::isArmMoving(armSide side, int dof)
{
    bool ret = false;
    static int l7debounce_count  = 0;
    static int l10debounce_count = 0;
    static int r7debounce_count  = 0;
    static int r10debounce_count = 0;

    // local debounce coiunt
    int debounce_count;
    armMotionState arm_state;

    // left arm - 10DOF
    if (side == armSide::LEFT && dof == 10 &&
            (track_torso_pitch_.isInMotion()     == frame_track_status::FRAME_IN_MOTION     ||
             track_torso_yaw_.isInMotion()       == frame_track_status::FRAME_IN_MOTION     ||
             track_torso_roll_.isInMotion()      == frame_track_status::FRAME_IN_MOTION     ||
             track_lshoulder_pitch_.isInMotion() == frame_track_status::FRAME_IN_MOTION     ||
             track_lshoulder_roll_.isInMotion()  == frame_track_status::FRAME_IN_MOTION     ||
             track_lshoulder_yaw_.isInMotion()   == frame_track_status::FRAME_IN_MOTION     ||
             track_lelbow_pitch_.isInMotion()    == frame_track_status::FRAME_IN_MOTION     ||
             track_lforearm_yaw_.isInMotion()    == frame_track_status::FRAME_IN_MOTION     ||
             track_lwrist_roll_.isInMotion()     == frame_track_status::FRAME_IN_MOTION     ||
             track_lwrist_pitch_.isInMotion()    == frame_track_status::FRAME_IN_MOTION))
    {
        debounce_count = l10debounce_count;
        arm_state = armMotionState::LEFT_10DOF;
    }

    // right arm - 10DOF
    if (side == armSide::RIGHT && dof == 10 &&
            (track_torso_pitch_.isInMotion()     == frame_track_status::FRAME_IN_MOTION     ||
             track_torso_yaw_.isInMotion()       == frame_track_status::FRAME_IN_MOTION     ||
             track_torso_roll_.isInMotion()      == frame_track_status::FRAME_IN_MOTION     ||
             track_rshoulder_pitch_.isInMotion() == frame_track_status::FRAME_IN_MOTION     ||
             track_rshoulder_roll_.isInMotion()  == frame_track_status::FRAME_IN_MOTION     ||
             track_rshoulder_yaw_.isInMotion()   == frame_track_status::FRAME_IN_MOTION     ||
             track_relbow_pitch_.isInMotion()    == frame_track_status::FRAME_IN_MOTION     ||
             track_rforearm_yaw_.isInMotion()    == frame_track_status::FRAME_IN_MOTION     ||
             track_rwrist_roll_.isInMotion()     == frame_track_status::FRAME_IN_MOTION     ||
             track_rwrist_pitch_.isInMotion()    == frame_track_status::FRAME_IN_MOTION))
    {
        debounce_count = r10debounce_count;
        arm_state = armMotionState::RIGHT_10DOF;
    }

    // left arm - 7DOF
    if (side == armSide::LEFT && dof == 7 &&
            (track_lshoulder_pitch_.isInMotion() == frame_track_status::FRAME_IN_MOTION     ||
             track_lshoulder_roll_.isInMotion()  == frame_track_status::FRAME_IN_MOTION     ||
             track_lshoulder_yaw_.isInMotion()   == frame_track_status::FRAME_IN_MOTION     ||
             track_lelbow_pitch_.isInMotion()    == frame_track_status::FRAME_IN_MOTION     ||
             track_lforearm_yaw_.isInMotion()    == frame_track_status::FRAME_IN_MOTION     ||
             track_lwrist_roll_.isInMotion()     == frame_track_status::FRAME_IN_MOTION     ||
             track_lwrist_pitch_.isInMotion()    == frame_track_status::FRAME_IN_MOTION))
    {
        debounce_count = l7debounce_count;
        arm_state = armMotionState::LEFT_7DOF;
    }

    // right arm - 7DOF
    if (side == armSide::RIGHT && dof == 7 &&
            (track_rshoulder_pitch_.isInMotion()   == frame_track_status::FRAME_IN_MOTION ||
             track_rshoulder_roll_.isInMotion()   == frame_track_status::FRAME_IN_MOTION  ||
             track_rshoulder_yaw_.isInMotion()   == frame_track_status::FRAME_IN_MOTION   ||
             track_relbow_pitch_.isInMotion()   == frame_track_status::FRAME_IN_MOTION    ||
             track_rforearm_yaw_.isInMotion()   == frame_track_status::FRAME_IN_MOTION    ||
             track_rwrist_roll_.isInMotion()   == frame_track_status::FRAME_IN_MOTION     ||
             track_rwrist_pitch_.isInMotion()   == frame_track_status::FRAME_IN_MOTION))
    {
        debounce_count = r7debounce_count;
        arm_state = armMotionState::RIGHT_7DOF;
    }


    // if the required arm group is in motion
    if (arm_state == armMotionState::LEFT_7DOF  ||
            arm_state == armMotionState::LEFT_10DOF ||
            arm_state == armMotionState::RIGHT_7DOF ||
            arm_state == armMotionState::RIGHT_10DOF)
    {
        // arm in motion
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

    // set the debounce count to static variable, if the arm moved
    if (arm_state == armMotionState::LEFT_7DOF)   l7debounce_count  = debounce_count;
    if (arm_state == armMotionState::LEFT_10DOF)  l10debounce_count = debounce_count;
    if (arm_state == armMotionState::RIGHT_7DOF)  r7debounce_count  = debounce_count;
    if (arm_state == armMotionState::RIGHT_10DOF) r10debounce_count = debounce_count;

    return ret;
}

bool uuperBodyTracker::isTorsoMoving(void)
{
    bool ret = false;
    static int debounce_count = 0;

    // std::cout << debounce_count << std::endl;

    // if the robot is walking
    if(track_torso_pitch_.isInMotion() == frame_track_status::FRAME_IN_MOTION ||
            track_torso_yaw_.isInMotion()   == frame_track_status::FRAME_IN_MOTION ||
            track_torso_roll_.isInMotion()  == frame_track_status::FRAME_IN_MOTION)
    {
        // torso is moving
        ret = true;

        // set the debounce count
        debounce_count = DEBOUNCE_COUNT;

        std::cout << "torso motion"<< std::endl;
    }
    // simple up down debounce for the transition from motion status to stationary status
    else if (debounce_count > 0)
    {
        // decrement the debounce count
        debounce_count--;

        // consider torso in motion (in the debounce period)
        ret = true;

        std::cout << "torso debounce motion"<< std::endl;
    }
    else
    {
        // reset the debounce count
        debounce_count = 0;

        // torso is not moving, its static
        ret = false;

        std::cout << "torso static" <<std::endl;
    }

    return ret;
}
