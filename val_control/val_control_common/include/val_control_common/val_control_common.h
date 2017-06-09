#pragma once

#include <ros/ros.h>
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/val_chest_navigation.h>
#include <val_controllers/val_pelvis_navigation.h>

class valControlCommon {
private:
    ros::NodeHandle nh_;

    ros::Publisher stop_traj_pub_;
    armTrajectory armTraj;
    chestTrajectory chestTraj;
    pelvisTrajectory pelvisTraj;

public:
    valControlCommon(ros::NodeHandle nh);
    ~valControlCommon();

    void stopAllTrajectories(void);
    void resetRobot();
};
