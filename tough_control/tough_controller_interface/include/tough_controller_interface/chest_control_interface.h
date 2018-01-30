#ifndef CHEST_CONTROL_INTERFACE_H
#define CHEST_CONTROL_INTERFACE_H

#include <ros/ros.h>
#include <ihmc_msgs/ChestTrajectoryRosMessage.h>
#include <tf/tf.h>
#include <math.h>
#include <time.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include "tough_controller_interface/robot_state.h"
#include "tough_common/robot_description.h"
#include "tough_controller_interface/tough_controller_interface.h"

class ChestControlInterface {

private:

    ros::NodeHandle nh_;
    ros::Publisher chestTrajPublisher_;
    RobotStateInformer *state_informer_;
    RobotDescription * rd_;
public:

    ChestControlInterface(ros::NodeHandle nh);
    ~ChestControlInterface();
    void controlChest(float roll , float pitch , float yaw, float time = 1.0f);
    void controlChest(geometry_msgs::Quaternion quat, float time = 1.0f);
};

#endif // CHEST_CONTROL_INTERFACE_H
