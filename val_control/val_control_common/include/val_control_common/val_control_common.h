#pragma once

#include <ros/ros.h>
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>

class valControlCommon {
private:
    ros::NodeHandle nh_;

    ros::Publisher stop_traj_pub_;

public:
    valControlCommon(ros::NodeHandle nh);
    ~valControlCommon();

    void stopAllTrajectories(void);
};
