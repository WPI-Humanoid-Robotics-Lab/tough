#pragma once

#include <ihmc_msgs/PelvisHeightTrajectoryRosMessage.h>
#include <ros/ros.h>

class pelvisTrajectory {

private:

    ros::NodeHandle nh_;
    ros::Publisher pelvisHeightPublisher;

public:

    pelvisTrajectory(ros::NodeHandle nh);
    ~pelvisTrajectory();
    void controlPelvisHeight(float height);
};
