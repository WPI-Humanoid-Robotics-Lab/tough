# pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <thread>

class FallDetector {

private:
    ros::NodeHandle nh_;
    std::string foot_frame_, root_frame_, world_frame_;
    bool isrobot_fallen_;

    tf::TransformListener foot_listener_, root_listener_;
    std::thread falltrack_thread_;

    void trackRobotFall(void);
    bool isTranformChanging (tf::StampedTransform transform_curr, tf::StampedTransform transform_prev);
    const float ZTHRESHOLD = 0.2f;

public:
    FallDetector(ros::NodeHandle nh, std::string foot_frame, std::string root_frame, std::string world_frame);
    ~FallDetector();

    bool isRobotFallen(void);
};
