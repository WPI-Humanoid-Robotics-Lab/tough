//Author: Syon Khosla
//Last edited: April 26th, 2018
//COMPLETED

#include <tough_controller_interface/chest_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of ChestControlInterface
    ChestControlInterface chestTraj(nh);
    float roll = 10;
    float pitch = 10;
    float yaw = 30;
    float duration = 5.0f; 

    // change the chest orientation. This is a non-blocking call.
    chestTraj.controlChest(roll, pitch, yaw, duration);

    // wait for the robot to move
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}
