#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <fstream>

ros::Subscriber sub;
std::ofstream file;

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    for (std::size_t i=0; i<msg->name.size(); i++)
    {
        file << msg->position[i] << ", " <<  msg->velocity[i] << ", " << msg->effort[i] << ",";
        //std::cout << msg->position[i];
    }
    file << "\n";
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "joint_states");
    ros::NodeHandle nh;
    //pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    sub = nh.subscribe("/joint_states", 1000, jointCallback);

    ros::Rate r(10);

    file.open("/home/sumanth/indigo_ws/src/valkyrie_control/dynamics.txt");
    while(ros::ok())
    {
        ros::spin();
        r.sleep();
    }
}
