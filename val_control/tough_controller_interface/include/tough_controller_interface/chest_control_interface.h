#ifndef VAL_CHEST_NAVIGATION_H
#define VAL_CHEST_NAVIGATION_H

#include <ros/ros.h>
#include <ihmc_msgs/ChestTrajectoryRosMessage.h>
#include <tf/tf.h>
#include <math.h>
#include <time.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include "tough_controller_interface/robot_state.h"
#include "val_common/robot_description.h"

class chestTrajectory {

private:

    ros::NodeHandle nh_;
    ros::Publisher chestTrajPublisher_;
    RobotStateInformer *state_informer_;
    RobotDescription * rd_;
public:

    chestTrajectory(ros::NodeHandle nh);
    ~chestTrajectory();
    void controlChest(float roll , float pitch , float yaw, float time = 1.0f);
};

#endif // VAL_CHEST_NAVIGATION_H
