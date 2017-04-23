# pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <thread>
#include <mutex>

// threshodl for determining the motion
#define POSITION_THRESHOLD     0.01        //0.1cm
#define ORIENTATION_THRESHOLD  0.0174533   // 1deg

enum class frame_track_status
{
    NOT_TRACKED,
    FRAME_IN_MOTION,
    FRAME_STATIONARY
};

class frameTracking {

private:
    tf::TransformListener listener_;
    ros::NodeHandle nh_;
    std::string frame_, base_frame_;
    frame_track_status motion_status_;
    std::thread track_thread_;
    std::mutex mtx_;

    void trackFrame(void);
    bool isTranformChanging (tf::StampedTransform transform_curr, tf::StampedTransform transform_prev);

public:
    frameTracking(ros::NodeHandle nh, std::string frame, std::string base_frame);
    ~frameTracking();

    frame_track_status isInMotion(void);
};
