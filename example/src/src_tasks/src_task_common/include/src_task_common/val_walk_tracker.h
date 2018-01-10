#include <navigation_common/frame_tracking.h>


#define DEBOUNCE_COUNT 10 // debounces the motion to stationary transition (its a up dopwn debounce logic)

class walkTracking{
private:
    ros::NodeHandle nh_;
    // object for motion tracking
    frameTracking track_pelvis_, track_leftFoot_, track_rightFoot_;

public:
    walkTracking(ros::NodeHandle nh);
    ~walkTracking();

    // determines if robot is waliking, returns True if walking
    bool isWalking(void);
};
