#ifndef VAL_PELVIS_NAVIGATION_H
#define VAL_PELVIS_NAVIGATION_H

#include <ihmc_msgs/PelvisHeightTrajectoryRosMessage.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "tough_common/robot_state.h"
#include "tough_common/robot_description.h"
#include "tough_controller_interface/tough_controller_interface.h"

class PelvisControlInterface : public ToughControllerInterface{

private:
    ros::Publisher pelvisHeightPublisher_;

public:

    PelvisControlInterface(ros::NodeHandle nh);
    ~PelvisControlInterface();
    void controlPelvisHeight(float height, float duration=2.0f);
    void publishPelvisMessage(const ihmc_msgs::PelvisHeightTrajectoryRosMessage &msg) const ;
};

#endif
