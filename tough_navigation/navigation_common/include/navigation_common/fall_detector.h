# pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <thread>
#include "tough_common/robot_state.h"
#include "tough_common/robot_description.h"

class FallDetector {

private:
    bool isrobot_fallen_;
    std::string foot_frame_, root_frame_, world_frame_;
    std::thread falltrack_thread_;

    ros::NodeHandle nh_;
    RobotStateInformer *current_state_;
    RobotDescription *rd_;

    void trackRobotFall(void);
    const float ZTHRESHOLD = 0.2f;

public:
    FallDetector(ros::NodeHandle nh);
    FallDetector(ros::NodeHandle nh, std::string foot_frame, std::string root_frame, std::string world_frame);
    ~FallDetector();

    bool isRobotFallen(void);
};
