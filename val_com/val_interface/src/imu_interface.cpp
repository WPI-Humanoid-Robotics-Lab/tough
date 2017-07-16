#include<val_interface/imu_interface.h>

// constructor


imu::imu(ros::NodeHandle &nh, const std::string imuTopic):
    nh_(nh), imuTopic_(imuTopic)
{
    sub_imu_ = nh.subscribe(imuTopic_, 1, &imu::imu_callback, this);
}

void imu::imu_callback(const sensor_msgs::Imu& msg_in)
{
    { //acquire the lock
        std::lock_guard<std::mutex> guard(mtx_);
        msg_ = msg_in;
    }
}

sensor_msgs::Imu imu::getIMU(void)
{
    { //acquire the lock
        std::lock_guard<std::mutex> guard(mtx_);
        return  msg_;
    }
}
