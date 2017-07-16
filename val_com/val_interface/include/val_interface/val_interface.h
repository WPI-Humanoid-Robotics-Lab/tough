#pragma once

#include<val_interface/imu_interface.h>
#include<val_interface/forcesensor_interface.h>

class valInterface {

private:

    // node handle
    ros::NodeHandle nh_;

    imu* pelvisMiddle_;
    imu* pelvisRear_;
    imu* leftTorso_;
    forcesensor* force_torque_states_;

public:
    valInterface (ros::NodeHandle& nh);
    ~valInterface();

    sensor_msgs::Imu getPelvisMiddleImu(void);
    sensor_msgs::Imu getPelvisRearIMU(void);
    sensor_msgs::Imu getLeftTorsoIMU(void);
    geometry_msgs::Wrench getForceSensLeftFoot(void);
     geometry_msgs::Wrench getForceSensLeftFootOffset(void);
     geometry_msgs::Wrench getForceSensRightFoot(void);
      geometry_msgs::Wrench getForceSensRightFootOffset(void);
};
