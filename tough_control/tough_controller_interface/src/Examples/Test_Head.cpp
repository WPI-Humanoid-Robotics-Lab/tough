//Author: Syon Khosla
//Date (of last edit): April 14th, 2018
//COMPLETED

#include <tough_controller_interface/head_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    //Declaring Head Control Interface object
    HeadControlInterface h(nh);

    //Setting roll, pitch, and yaw
    float roll = 30;
    float pitch = 15;
    float yaw = 10;
    float time = 3;
    
    //Moves head
    h.moveHead(roll, pitch, yaw);

    //Sleeps, waiting for motion to complete
    ros::Duration(2).sleep(); 
    ROS_INFO("Motion finished");

    return 0;
}
