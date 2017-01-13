#pragma once
#include <ros/ros.h>
#include <ihmc_msgs/ChestTrajectoryRosMessage.h>
#include <tf/tf.h>
#include <math.h>
#include <time.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
class chestTrajectory {

private:

    ros::NodeHandle nh_;
    ros::Publisher chestTrajPublisher;

public:

    chestTrajectory(ros::NodeHandle nh);
    ~chestTrajectory();
    void controlchest(float roll , float pitch , float yaw);
};
