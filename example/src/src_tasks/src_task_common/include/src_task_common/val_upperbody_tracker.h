# pragma once

#include <navigation_common/frame_tracking.h>
#include <tough_common/val_common_defines.h>

#define DEBOUNCE_COUNT 10 // debounces the motion to stationary transition (its a up dopwn debounce logic)

// arm states
enum class armMotionState {
    LEFT_7DOF = 0,
    LEFT_10DOF,
    RIGHT_7DOF,
    RIGHT_10DOF
};

class upperBodyTracker{
private:
    ros::NodeHandle nh_;
    // object for motion tracking
    frameTracking
    // left arm trackers
    track_lshoulder_pitch_, track_lshoulder_roll_, track_lshoulder_yaw_, track_lelbow_pitch_, track_lforearm_yaw_, track_lwrist_roll_, track_lwrist_pitch_,
    // right arm trackers
    track_rshoulder_pitch_, track_rshoulder_roll_, track_rshoulder_yaw_, track_relbow_pitch_, track_rforearm_yaw_, track_rwrist_roll_, track_rwrist_pitch_,
    // torso trackers
    track_torso_pitch_,
    track_torso_yaw_,
    track_torso_roll_;

public:
    upperBodyTracker(ros::NodeHandle nh);
    ~upperBodyTracker();

    // determines if robot is waliking, returns True if walking
    bool isArmMoving(RobotSide side, int dof);
    bool isTorsoMoving(void);
};
