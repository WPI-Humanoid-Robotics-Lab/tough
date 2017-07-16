#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mutex>

class imu {
private:

    // node handle
    ros::NodeHandle nh_;
    // subscriber
    ros::Subscriber sub_imu_;

    const std::string imuTopic_;
    sensor_msgs::Imu msg_;
    std::mutex mtx_;
    void imu_callback(const sensor_msgs::Imu& msg_in);

public:
    imu (ros::NodeHandle& nh, const std::string imuTopic);
    ~imu();
    // methods
    sensor_msgs::Imu getIMU(void);
};
