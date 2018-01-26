#ifndef TOUGHCONTROLLERINTERFACE_H
#define TOUGHCONTROLLERINTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ihmc_msgs/WholeBodyTrajectoryRosMessage.h>
#include "tough_controller_interface/robot_state.h"
#include "tough_common/robot_description.h"


class ToughControllerInterface
{

protected:
    ros::NodeHandle nh_;
    static long id_;
    RobotStateInformer *state_informer_;
    RobotDescription *rd_;
    std::string control_topic_prefix_;
    std::string output_topic_prefix_;

public:

    ToughControllerInterface(ros::NodeHandle nh);
    virtual ~ToughControllerInterface()=0;

};

#endif // TOUGHCONTROLLERINTERFACE_H
