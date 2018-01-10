#ifndef VAL_PELVIS_NAVIGATION_H
#define VAL_PELVIS_NAVIGATION_H

#include <ihmc_msgs/PelvisHeightTrajectoryRosMessage.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "tough_controller_interface/robot_state.h"
#include "tough_common/robot_description.h"

class PelvisControlInterface {

private:

    ros::NodeHandle nh_;
    ros::Publisher pelvisHeightPublisher_;
    static int pelvis_id_;
    RobotStateInformer *state_informer_;
    RobotDescription *rd_;

public:

    PelvisControlInterface(ros::NodeHandle nh);
    ~PelvisControlInterface();
    void controlPelvisHeight(float height);
    bool controlPelvisMessage(ihmc_msgs::PelvisHeightTrajectoryRosMessage msg);
};

#endif
