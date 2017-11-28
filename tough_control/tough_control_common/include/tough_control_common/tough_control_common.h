#pragma once

#include <ros/ros.h>
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <tough_controller_interface/pelvis_control_interface.h>

class valControlCommon {
private:
    ros::NodeHandle nh_;

    ros::Publisher stop_traj_pub_;
    ArmControlInterface armTraj;
    ChestControlInterface chestTraj;
    PelvisControlInterface pelvisTraj;

public:
    valControlCommon(ros::NodeHandle nh);
    ~valControlCommon();

    void stopAllTrajectories(void);
    void resetRobot();
};
