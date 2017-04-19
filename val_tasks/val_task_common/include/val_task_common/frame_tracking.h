# pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>

// threshodl for determining the motion
#define POSITION_THRESHOLD     0.02        //2cm
#define ORIENTATION_THRESHOLD  0.0174533   // 1deg

enum frame_track_status
{
    NOT_TRACKED,
    FRAME_IN_MOTION,
    FRAME_STATIONARY
};

class frameTracking {

private:
    ros::NodeHandle nh_;
    std::string frame_;
    frame_track_status motion_status_;
    void trackFrame(std::string frame);
    bool isTranformChanging (tf::StampedTransform transform_curr, tf::StampedTransform transform_prev);

public:
    frameTracking(ros::NodeHandle nh, std::string frame);
    ~frameTracking();

    frame_track_status isInMotion(void);
};
