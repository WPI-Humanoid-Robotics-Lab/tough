#pragma once

#include <ros/ros.h>
#include <mutex>
#include <sensor_msgs/JointState.h>

class valJointState{

private:

    //node handle

    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    ros::Subscriber hw_joint_state_sub_, joint_state_sub_;

    sensor_msgs::JointState hw_jointstate_, joint_state_;

    // mutex to handle the shared variable of joint states
    std::mutex jointcb_mutex_;

public:
    void hwjointStateCb(const sensor_msgs::JointState::ConstPtr& msg);
    sensor_msgs::JointState fetchJointState();
    void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg);
    sensor_msgs::JointState getJointStates(void);


    valJointState(ros::NodeHandle nh);
    ~valJointState();


};
